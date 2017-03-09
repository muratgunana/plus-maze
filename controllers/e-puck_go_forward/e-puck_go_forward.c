#include "genotype.h"
#include "population.h"
#include "random.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/display.h>
#include <webots/keyboard.h>

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

// time in [ms] of a simulation step
#define TIME_STEP 64

// set neuron numbers and inputs here
static const int NEURON_INPUTS = 3;
static const int NUMBER_OF_NEURONS = 6;
double weights[NUMBER_OF_NEURONS][NEURON_INPUTS];
double inputs[2];

static const int POPULATION_SIZE = 50;
static const int NUM_GENERATIONS = 1;
//static const char *FILE_NAME = "fittest.txt";

#define GENOTYPE_SIZE (NEURON_INPUTS*NUMBER_OF_NEURONS)

// the GA population
static Population population;

// setup neural network.
void setup_neural_net() {
  // initialize weights including bias.
  int i,j;
  int size = sizeof(weights) / sizeof(double);
  int size_x = size / (sizeof(weights[0]) / sizeof(double));
  int size_y = size / size_x;
  for (i = 0; i < size_x; ++i) {
    for (j = 0; j < size_y; ++j) {
      if (j == 0) {
        weights[i][j] = 1.0;
      }
      else {
        weights[i][j] = (rand() % 202 - 101) / 100.0;
      }
      printf("Weights %f\n", weights[i][j]);
    }
  }
}

// sguash the ANN output between -1 and 1.
double hyperbolic_tangent(double value) {
  return (1.0f - exp(- 2.0f * value)) / (1.0f + exp(-2.0f * value));
}

// Calculate the output based on weights evolved by GA.
double* evolve_neural_net(Genotype genotype) {
  double nn_weights[NUMBER_OF_NEURONS][NEURON_INPUTS];
  const double* genes = genotype_get_genes(genotype);
  int m,n;
  
  // Copy genes as an array in nn_weights format.
  for (m = 0; m < NUMBER_OF_NEURONS; ++m) {
    for (n = 0; n < NEURON_INPUTS; ++n) {
      nn_weights[m][n] = genes[NEURON_INPUTS * m + n];
    }
  }
  int input_size = sizeof(inputs) / sizeof(double);
  int i,j, k;
  double* h;
  h = malloc(input_size);
  memcpy(h, inputs, input_size);
  
  for (i = 0; i < input_size; ++i) {
    h[i] = hyperbolic_tangent(h[i]);
  }
  
  for (i = 0; i < NUMBER_OF_NEURONS; ++i) {
    for (k = 0; k < input_size; ++k) {
      double output = 0.0;
      for (j = 0; j < NEURON_INPUTS; ++j) {
        
        if (j == 0) {
          output += nn_weights[i][j];
        }
        else {
          output += nn_weights[i][j] * h[j-1];
        }
      }
      
      h[k] = output;
    }
  }
  
  for (i = 0; i < input_size; ++i) {
    h[i] = hyperbolic_tangent(h[i]);
  }
  return h;
}

// compute fitness as the euclidian distance that the load was pushed
double measure_fitness(double *values) {
  double V,deltaV;
  
  // Sum of rotation speeds between two wheels.
  V = fabs(values[0]) + fabs(values[1]);
  
  // Add +1 to each speed value in order bring them between [0,2].
  // This encourages the wheels to move in the same direction.
  deltaV = fabs((values[0] + 1.0f) - (values[1] + 1.0f));
  printf("V: %f\n", V);
  printf("deltaV: %f\n", deltaV);
  // Computes all the values together.
  return V * (2.0f - sqrt(deltaV)) 
  * (2.0f - ((hyperbolic_tangent(inputs[0]) 
  + hyperbolic_tangent(inputs[1])) / 2.0f));
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {
  double* output = evolve_neural_net(genotype);
  // measure fitness
  double fitness = measure_fitness(output);
  printf("Actual fitness: %f\n", fitness);
  genotype_set_fitness(genotype, fitness);

  printf("fitness: %g\n", fitness);
}

void run_optimization() {
  wb_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);

  int i, j;
  for  (i = 0; i < NUM_GENERATIONS; i++) {
    for (j = 0; j < POPULATION_SIZE; j++) {
      printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype);
    }

    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    //double average_fitness = population_compute_average_fitness(population);

    // display results
    //plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    //printf("average fitness: %g\n", average_fitness);

    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }

  printf("GA optimization terminated.\n");

  //population_destroy(population);
}

// entry point of the controller
int main(int argc, char **argv)
{
  // initialize the Webots API
  wb_robot_init();

  // internal variables
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  // initialize devices
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  //setup_neural_net();
  
  // get simulation step in milliseconds
  //time_step = wb_robot_get_basic_time_step();
// initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
  
  //if (demo)
    //run_demo();
  
  
  // feedback loop
  while (1) {
    // step simulation
    int delay = wb_robot_step(TIME_STEP);
    if (delay == -1) // exit event from Webots
      break;

    // read sensors outputs
    double ps_values[8];
    
    for (i=0; i<8 ; i++) {
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
      
      if (i == 0) {
        inputs[0] = ps_values[i];
      }
      else if (i == 7) {
        inputs[1] = ps_values[i];
      }
      //printf("sensor %d: %f\n", i, ps_values[i]);
      //printf("Sensor %f\n",inputs[i]);
    }
    
    // detect obstacles
    bool right_obstacle =
      ps_values[0] > 100.0 ||
      ps_values[1] > 100.0 ||
      ps_values[2] > 100.0;
    bool left_obstacle =
      ps_values[5] > 100.0 ||
      ps_values[6] > 100.0 ||
      ps_values[7] > 100.0;

    // init speeds
    double left_speed  = 500;
    double right_speed = 500;

    // modify speeds according to obstacles
    if (left_obstacle) {
      left_speed  += 500;
      right_speed -= 500;
    }
    else if (right_obstacle) {
      left_speed  -= 500;
      right_speed += 500;
    }

    // write actuators inputs
    wb_differential_wheels_set_speed(left_speed, right_speed);

    
    //run GA optimization
    run_optimization();
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
