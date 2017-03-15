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
#define TIME_STEP 512

// set neuron numbers and inputs here
static const int NUMBER_OF_INPUTS = 8;
static const int INPUT_LAYER_NUMBER_OF_NEURONS = 20;
static const int HIDDEN_LAYER_NUMBER_OF_NEURONS = 20;
static const int OUTPUT_LAYER_NUMBER_OF_NEURONS = 2;

int inputs[NUMBER_OF_INPUTS];

static const int POPULATION_SIZE = 20;
static const int NUM_GENERATIONS = 10000;
//static const char *FILE_NAME = "fittest.txt";

#define GENOTYPE_SIZE ((NUMBER_OF_INPUTS + 1) * INPUT_LAYER_NUMBER_OF_NEURONS + (INPUT_LAYER_NUMBER_OF_NEURONS + 1) * HIDDEN_LAYER_NUMBER_OF_NEURONS + (HIDDEN_LAYER_NUMBER_OF_NEURONS * HIDDEN_LAYER_NUMBER_OF_NEURONS) + (HIDDEN_LAYER_NUMBER_OF_NEURONS + 1) * OUTPUT_LAYER_NUMBER_OF_NEURONS)

double recurrent_inputs[GENOTYPE_SIZE][HIDDEN_LAYER_NUMBER_OF_NEURONS];
//#define GENOTYPE_SIZE (18)
// the GA population
static Population population;

// Initialize hidden layer recurrent neural net inputs.
void init_recurrent_inputs() {
  int i, j;
  for (i = 0; i < GENOTYPE_SIZE; ++i) {
    for (j = 0; j < HIDDEN_LAYER_NUMBER_OF_NEURONS; ++j) {
      recurrent_inputs[i][j] = random_get_uniform();
      //printf("Recurrent input[%d]: %f\n", i, recurrent_inputs[i]); 
    }
  }
}

// Calculate the output based on weights evolved by GA.
double* evolve_neural_net(Genotype genotype, int genotype_index) {
  const double* gen = malloc(GENOTYPE_SIZE);
  gen = genotype_get_genes(genotype);
 
  int i,j;
  //double *input_layer_outputs,
  //*hidden_layer_outputs, 
  //*output_layer_outputs;
  
  static double input_layer_outputs[INPUT_LAYER_NUMBER_OF_NEURONS],
  hidden_layer_outputs[HIDDEN_LAYER_NUMBER_OF_NEURONS], 
  output_layer_outputs[OUTPUT_LAYER_NUMBER_OF_NEURONS];
  //input_layer_outputs = malloc(INPUT_LAYER_NUMBER_OF_NEURONS);
  //hidden_layer_outputs = malloc(HIDDEN_LAYER_NUMBER_OF_NEURONS);
  //output_layer_outputs = malloc(OUTPUT_LAYER_NUMBER_OF_NEURONS);
  
  for (i = 0; i < GENOTYPE_SIZE; ++i) {
    //recurrent_inputs[i] = random_get_uniform();
    //printf("Genes input[%d]: %f\n", i, genes[i]); 
  }
  
  // Input layer.
  for (i = 0; i < INPUT_LAYER_NUMBER_OF_NEURONS; ++i) {
    
    double output = 0.0f;
    for (j = 0; j < NUMBER_OF_INPUTS + 1; ++j) {
      if (j == 0) {
        output += gen[NUMBER_OF_INPUTS * i + i + j];
        //printf("gene: [%d]\n",NUMBER_OF_INPUTS * i + i + j);
      }
      else {
        //printf("gene: [%d]\n",NUMBER_OF_INPUTS * i + i + j);
        output += gen[NUMBER_OF_INPUTS * i + i + j] * tanh(inputs[j-1]);
      }        
    }
    //printf("input_layer_outputs[%d]: %f\n",i, output);
    input_layer_outputs[i] = tanh(output);
    //printf("input_layer_outputs2[%d]: %f\n",i, input_layer_outputs[i]);
  }
  //printf("input_layer_outputs[0]: %f\n", input_layer_outputs[0]);
  //printf("input_layer_outputs[1]: %f\n", input_layer_outputs[1]);
  
  // Hidden layer.
  int  gene_offset = (NUMBER_OF_INPUTS + 1) * INPUT_LAYER_NUMBER_OF_NEURONS;
  int hidden_layer_inputs = INPUT_LAYER_NUMBER_OF_NEURONS + HIDDEN_LAYER_NUMBER_OF_NEURONS;
  for (i = 0; i < HIDDEN_LAYER_NUMBER_OF_NEURONS; ++i) {
    
    double output = 0.0f;
    for (j = 0 ; j < INPUT_LAYER_NUMBER_OF_NEURONS + 1; ++j) {
      if (j == 0) {
        output += gen[gene_offset + hidden_layer_inputs * i + i + j];
        //printf("gene: [%d]\n", gene_offset + (hidden_layer_inputs) * i + i + j);
      }
      else {
        //printf("gene: [%d]\n", gene_offset + hidden_layer_inputs * i + i + j);
        output += gen[gene_offset + hidden_layer_inputs * i + i + j] * input_layer_outputs[j-1];
      }        
    }
    //printf("hidden_layer_outputs[%d]: %f\n",i, output);
    // Elmar neural network implementation.
    int k;
    for (k = j ; k < HIDDEN_LAYER_NUMBER_OF_NEURONS + j ; ++k) {
      //printf("gene: [%d]\n", gene_offset + hidden_layer_inputs * i + i + k);
      output += gen[gene_offset + hidden_layer_inputs * i + i + k] * tanh(recurrent_inputs[genotype_index][k]);      
    } 
    //printf("hidden_layer_outputs[%d]: %f\n",i, output);
    hidden_layer_outputs[i] = tanh(output);
    //printf("hidden_layer_outputs2[%d]: %f\n",i, hidden_layer_outputs[i]);
  }
  //printf("hidden_layer_outputs[0]: %f\n", hidden_layer_outputs[0]);
  //printf("hidden_layer_outputs[1]: %f\n", hidden_layer_outputs[1]);
  
  // Save hidden layer outputs as recurrent inputs to be used next time.
  for (i = 0 ; i < HIDDEN_LAYER_NUMBER_OF_NEURONS ; ++i) {
     recurrent_inputs[genotype_index][i] = hidden_layer_outputs[i];    
  } 
  
  // Output layer.
  gene_offset += (INPUT_LAYER_NUMBER_OF_NEURONS + 1) * HIDDEN_LAYER_NUMBER_OF_NEURONS 
  + (HIDDEN_LAYER_NUMBER_OF_NEURONS * HIDDEN_LAYER_NUMBER_OF_NEURONS);
      
  for (i = 0; i < OUTPUT_LAYER_NUMBER_OF_NEURONS; ++i) {
    
    double output = 0.0f;
    for (j = 0; j < HIDDEN_LAYER_NUMBER_OF_NEURONS + 1; ++j) {
      if (j == 0) {
        output += gen[gene_offset + HIDDEN_LAYER_NUMBER_OF_NEURONS * i + i + j];
        //printf("gene: [%d]\n", gene_offset + HIDDEN_LAYER_NUMBER_OF_NEURONS * i + i + j);
      }
      else {
        output += gen[gene_offset + HIDDEN_LAYER_NUMBER_OF_NEURONS * i + i + j] * hidden_layer_outputs[j-1];
        //printf("gene: [%d]\n", gene_offset + HIDDEN_LAYER_NUMBER_OF_NEURONS * i + i + j);
      }        
    }
    //printf("output_layer_outputs[%d]: %f\n",i, output);
    output_layer_outputs[i] = tanh(output);
    //printf("output_layer_outputs2[%d]: %f\n",i, output_layer_outputs[i]);
  }
  //printf("output_layer_outputs[0]: %f\n", output_layer_outputs[0]);
  //printf("output_layer_outputs[1]: %f\n", output_layer_outputs[1]);
  return output_layer_outputs;
}

// compute fitness as the euclidian distance that the load was pushed
double measure_fitness(double *values) {
  double V,deltaV;
  
  // Sum of rotation speeds between two wheels.
  V = fabs(values[0]) + fabs(values[1]);
  
  // Add +1 to each speed value in order bring them between [0,2].
  // This encourages the wheels to move in the same direction.
  deltaV = fabs((values[0] + 1) - (values[1] + 1));
  
  int i;
  int total = 0;
  for (i = 0; i < NUMBER_OF_INPUTS; ++i) {
    total += inputs[i];
  }
  total = total / (NUMBER_OF_INPUTS);
  //double obstacle_avoidance = total / 10.0f;
  //printf("Obstacle : %f\n", obstacle_avoidance);
  
     int max=-32000;
     for (i=0; i<NUMBER_OF_INPUTS; i++)
     {
  	 if (inputs[i]>max)
  	 {
  	    max=inputs[i];
  	 }
     }
     //printf("Values[0]: %f\n", values[0]);
     //printf("Values[1]: %f\n", values[1]);
     //printf("max: %d\n", max);
  if (max > 100){
      //printf("max: %d\n", max);
      //max = 200;
     return 0.9f;
     //deltaV = fabs(values[0]) + fabs(values[1]);
    //return V  * ( 2 - sqrt(deltaV)) * (1+ 1.0f/max);
  }
  //printf("Max-tanh: %f\n", 2.0f- 1.0f/max);
  // Compute all the components together.
  double fit = V  *  (2.0f - sqrt(deltaV)) * (100.0f/max);
  
  return fit;
}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype, int index) {
  double* output = evolve_neural_net(genotype, index);
  
  // measure fitness
  double fitness = measure_fitness(output);
  //printf("Actual fitness: %f\n", fitness);
  double current_fit = genotype_get_fitness(genotype);
  //if (fitness > current_fit) {
  genotype_set_fitness(genotype, fitness);
  //}
  

  //printf("fitness: %g\n", fitness);
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
  
  
  //if (demo)
    //run_demo();
  
  
  // feedback loop
  while (1) {
    

    
    population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
    //run GA optimization
    
    
    wb_keyboard_disable();

  printf("---\n");
  printf("starting GA optimization ...\n");
  printf("population size is %d, genome size is %d\n", POPULATION_SIZE, GENOTYPE_SIZE);
  
  
  
  
  int i, j, k;
  for  (i = 0; i < NUM_GENERATIONS; i++) {
    // initialize reccurent network inputs.
    init_recurrent_inputs();
    
    //printf("Num of gen: %d\n", i);
    
    // step simulation
    int delay = wb_robot_step(TIME_STEP);
    
    if (delay == -1) // exit event from Webots
      break;
      
    // read sensors outputs
    double ps_values[8];
    
    for (k=0; k<8 ; k++) {
      ps_values[k] = wb_distance_sensor_get_value(ps[k]);
      //if (i !=3 && i != 4)
      inputs[k] = ps_values[k];
      //if (i == 6) {
        //inputs[0] = ps_values[i];
        //printf("Sensor[%d] %d\n",i,inputs[0]);
      //}
      //else if (i == 7) {
        //inputs[1] = ps_values[i];
        //printf("Sensor[%d] %d\n",i,inputs[1]);
      //}
      //printf("sensor %d: %f\n", i, ps_values[i]);
      //printf("Sensor %f\n",inputs[i]);
    }
    
    for (j = 0; j < POPULATION_SIZE; j++) {
      //printf("generation: %d, genotype: %d\n", i, j);

      // evaluate genotype
      Genotype genotype = population_get_genotype(population, j);
      evaluate_genotype(genotype, i);
    }
    
    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    //double average_fitness = population_compute_average_fitness(population);

    // display results
    //plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    //printf("average fitness: %g\n", average_fitness);
    
    
    
     //printf("End\n");
    //Genotype g;
    
    double* output;
    output = evolve_neural_net(population_get_fittest(population), i); 
    // detect obstacles
    //printf("Output[%d]: %f\n", 0, output[0]);
    //printf("Output[%d]: %f\n", 1, output[1]);
    
    
    best_fitness = genotype_get_fitness(population_get_fittest(population));
    //double average_fitness = population_compute_average_fitness(population);

    // display results
    //plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness2: %g\n", best_fitness);
    // init speeds
    double left_speed  = output[0] * 1000;
    double right_speed = output[1] * 1000;
    
    // detect obstacles
    //bool right_obstacle =
      //inputs[0] > 100.0 ||
      //inputs[1] > 100.0 ||
      //inputs[2] > 100.0 ||
      //inputs[3] > 100.0;
    //bool left_obstacle =
      //inputs[4] > 100.0 ||
      //inputs[5] > 100.0 ||
      //inputs[6] > 100.0 ||
      //inputs[7] > 100.0;

    

    // modify speeds according to obstacles
    //if (left_obstacle) {
      //left_speed  += 150;
      //right_speed -= 150;
    //}
    //else if (right_obstacle) {
      //left_speed  -= 150;
      //right_speed +=150;
    //}

    // write actuators inputs
    wb_differential_wheels_set_speed(left_speed, right_speed);
    
    // reproduce (but not after the last generation)
    if (i < NUM_GENERATIONS - 1){
      population_reproduce(population);
      printf("Population reproduced. \n");
      }
    
    
  }
    population_destroy(population);
    
    if (i == NUM_GENERATIONS) {
    //break;
    }
    
   
  }

  // cleanup the Webots API
  wb_robot_cleanup();
  return 0; //EXIT_SUCCESS
}
