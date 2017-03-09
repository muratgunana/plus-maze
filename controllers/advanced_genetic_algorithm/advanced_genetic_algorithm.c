// Description:   Robot execution code for genetic algorithm

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <string.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define NUM_SENSORS 8

#define NEURON_INPUTS 3
#define NUMBER_OF_NEURONS 6
#define MAX_SPEED 1000
double weights[NUMBER_OF_NEURONS][NEURON_INPUTS];

#define GENOTYPE_SIZE (NEURON_INPUTS*NUMBER_OF_NEURONS)

double inputs[2];

WbDeviceTag sensors[NUM_SENSORS];  // proximity sensors
WbDeviceTag receiver;              // for receiving genes from Supervisor


// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double)) {
    const double* genes = wb_receiver_get_data(receiver);
    int m,n;
  
    // Copy genes as an array in nn_weights format.
    for (m = 0; m < NUMBER_OF_NEURONS; ++m) {
      for (n = 0; n < NEURON_INPUTS; ++n) {
        weights[m][n] = genes[NEURON_INPUTS * m + n];
        #ifdef DEBUG
        printf("Gene: %f\n",  weights[m][n]);
        #endif
      }
    }
 
  }
  
   // printf("Size received: %d \n", GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
//   memcpy(weights,nn_weights, 18);

    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver); 
}

// sguash the ANN output between -1 and 1.
double hyperbolic_tangent(double value) {
  printf("Value: %f\n", value);
  return (1.0f - exp(- 2.0f * value)) / (1.0f + exp(-2.0f * value));
}

// Calculate the output based on weights evolved by GA.
double* evolve_neural_net() {
  
  int input_size = sizeof(inputs) / sizeof(double);
  int i,j, k;
  double* h;
  h = malloc(input_size);
  memcpy(h, inputs, input_size);
  
  int m,n;
  // Copy genes as an array in nn_weights format.
  for (m = 0; m < NUMBER_OF_NEURONS; ++m) {
    for (n = 0; n < NEURON_INPUTS; ++n) {
      #ifdef DEBUG
      printf("Gene [%i,%i]: %f\n",  m,n,weights[m][n]);
      #endif 
    }
  }
  
  for (i = 0; i < input_size; ++i) {
    h[i] = hyperbolic_tangent(h[i]);
  }
  
  for (i = 0; i < NUMBER_OF_NEURONS; ++i) {
    for (k = 0; k < input_size; ++k) {
      double output = 0.0;
      for (j = 0; j < NEURON_INPUTS; ++j) {
        
        if (j == 0) {
          output += weights[i][j];
        }
        else {
          output += weights[i][j] * h[j-1];
        }
      }
      
      h[k] = output;
    }
  }
  
  for (i = 0; i < input_size; ++i) {
    h[i] = hyperbolic_tangent(h[i]);
    printf("H[%i]: %f\n", i , h[i]);
  }
  return h;
}

// Get input, evolve NN and move based on output
void sense_compute_and_actuate() {
  // read sensor values
  double sensor_values[NUM_SENSORS];
  int i;
  for (i = 0; i < NUM_SENSORS; i++)
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
  inputs[0] = sensor_values[0];
  inputs[1] = sensor_values[7];
  
  double * wheel_speeds = evolve_neural_net();
  
  // actuate e-puck wheels
  wb_differential_wheels_set_speed(wheel_speeds[0]*MAX_SPEED, wheel_speeds[1]*MAX_SPEED);
}

int main(int argc, const char *argv[]) {

  wb_robot_init();  // initialize Webots
  memset(weights, 0.0, sizeof(weights));
  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();

  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NUM_SENSORS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(sensors[i], time_step);
  }

  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);

  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }

  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
