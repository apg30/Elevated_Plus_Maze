//   Description:   Supervisor code for genetic algorithm

#include "genotype.h"
#include "population.h"
#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/display.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

static const int POPULATION_SIZE = 15;
static const int NUM_GENERATIONS = 30;
static const char *FILE_NAME = "fittest.txt";

// same variables as from the robot controller
static const int NUM_SENSORS = 8;
static const int NUM_WHEELS  = 2;
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

int max, danger, falling;  // for the fitness function
double oldx, oldy;         // for distance calculation
double furthestDistance, DistanceToStart, totaldistance;   // more distance vaariables
static double ground_sensors_values[3] = {300, 300, 300};

// index access
enum { X, Y, Z };

static int time_step;
// Device stuff for emitter and receiver
WbDeviceTag receiver;
static WbDeviceTag emitter;   // to send genes to robot
static WbDeviceTag display;   // to display the fitness evolution
static int display_width, display_height;

// the GA population
static Population population;

// for reading or setting the robot's position and orientation
static WbFieldRef robot_translation;
static WbFieldRef robot_rotation;
static double robot_trans0[3];  // a translation needs 3 doubles
static double robot_rot0[4];    // a rotation needs 4 doubles
    
//********************drawing the graph********************************
void draw_scaled_line(int generation, double y1, double y2) {
  const double XSCALE = (double)display_width / NUM_GENERATIONS;
  const double YSCALE = 0.1;
  wb_display_draw_line(display, (generation - 0.5) * XSCALE, display_height - y1 * YSCALE,
    (generation + 0.5) * XSCALE, display_height - y2 * YSCALE);
}

// plot best and average fitness
void plot_fitness(int generation, double best_fitness, double average_fitness) {
  static double prev_best_fitness = 0.0;
  static double prev_average_fitness = 0.0;
  if (generation > 0) {  
    wb_display_set_color(display, 0xff0000); // red
    draw_scaled_line(generation, prev_best_fitness, best_fitness);

    wb_display_set_color(display, 0x00ff00); // green
    draw_scaled_line(generation, prev_average_fitness, average_fitness);
  }

  prev_best_fitness = best_fitness;
  prev_average_fitness = average_fitness;
}
//*************************************************************************

// find simulation step in milliseconds
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int) wb_robot_get_basic_time_step();
  return time_step;
}

// check the robot stays inside the black area
static int cliff_detected() {
   if (ground_sensors_values[1] > 500)      // cliff in front
      return 1;
    else if (ground_sensors_values[0] > 500)// cliff on left
      return 0;
    else if (ground_sensors_values[2] > 500)// cliff on right
      return 2;
    else return 5; // no cliff
}

void check_node() {
      int c, d, cliff;

  // receive sensor data from the emitter
  if (wb_receiver_get_queue_length(receiver) > 0) {
      const double *array = wb_receiver_get_data(receiver);
      
      max = 0;         //reset max   
      max = array[0];  //set it to first element in array

      // get IR sensor data
      for (c = 0; c < 8; c++)
        if (array[c] > max)
           max = array[c];
        
      if (max < 50)  //then it is not next to a wall and needs to be punished
          danger = danger + 1; 

      // get ground sensor data
      for (d = 0; d < 3; d++)
        ground_sensors_values[d] = array[d + 8];
           
        wb_receiver_next_packet(receiver);
  }
         // robot is falling if ground sensors see white
         cliff = cliff_detected();
         if (cliff == 0 || cliff == 1 || cliff == 2)
             falling += 10;
               
         // save maximum displacement from start point
         if (DistanceToStart > furthestDistance)
            furthestDistance = DistanceToStart;    
}

// run the robot simulation for x seconds
void run_seconds(double seconds) {
      int i, n = 1000.0 * seconds / time_step;
      for (i = 0; i < n; i++) {
  
      // measure distance robot has travelled in this timestep and add it to the total  
      const double *load_trans = wb_supervisor_field_get_sf_vec3f(robot_translation);
      
      double dx = load_trans[X] - oldx;
      double dz = load_trans[Z] - oldy;
      totaldistance += sqrt(dx * dx + dz * dz);
      
      double dxf = load_trans[X] - robot_trans0[X];
      double dzf = load_trans[Z] - robot_trans0[Z];
      DistanceToStart = sqrt(dxf * dxf + dzf * dzf);
      
      oldx = load_trans[X];
      oldy = load_trans[Z];

      wb_robot_step(time_step);      // move on to next time step

      check_node();
  }
}

// calculate the fitness
double measure_fitness() {
   double fitness;
   
   // fitness is a function of TOTALDISTANCE + FURTHESTDISTANCE - DANGER - FALLING
   fitness = (totaldistance * 100) + (furthestDistance* 400) - (danger ) - (falling);
   printf("--------------\n");
   printf("danger : %d \n",danger);
   printf("distance: %f \n", totaldistance);
   printf("furthest distance: %f \n", furthestDistance);
   printf("fitness : %f \n",fitness);
   return fitness;

}

// evaluate one genotype at a time
void evaluate_genotype(Genotype genotype) {
  
  // send genotype to robot for evaluation
  wb_emitter_send(emitter, genotype_get_genes(genotype), GENOTYPE_SIZE * sizeof(double));
 //wb_emitter_send(emitter, matrix, GENOTYPE_SIZE * sizeof(double));
  
  // reset robot position and rotation
  wb_supervisor_field_set_sf_vec3f(robot_translation, robot_trans0);
  wb_supervisor_field_set_sf_rotation(robot_rotation, robot_rot0);
  
  // reset variables
  totaldistance = 0;
  danger = 0;
  furthestDistance = 0;
  DistanceToStart = 0;
  falling = 0;
  
  // reset the physics (fixes physics bug)
  wb_supervisor_node_reset_physics(wb_supervisor_node_get_from_def("ROBOT"));

  // evaluate genotype during x seconds
  run_seconds(300.0);  // 5 minutes
 
  // measure fitness (send fitness to the genotype class)
  double fitness = measure_fitness();
  genotype_set_fitness(genotype, fitness);
}

void run_optimization() {
  wb_robot_keyboard_disable();

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
  
    // get data for the graph
    double best_fitness = genotype_get_fitness(population_get_fittest(population));
    double average_fitness = population_compute_average_fitness(population);
    
    // display results
    plot_fitness(i, best_fitness, average_fitness);
    printf("best fitness: %g\n", best_fitness);
    printf("average fitness: %g\n", average_fitness);
    
    // reproduce (but not after the final generation)
    if (i < NUM_GENERATIONS - 1)
      population_reproduce(population);
  }
  
  printf("GA optimization terminated.\n");

  // save fittest individual
  Genotype fittest = population_get_fittest(population);
  FILE *outfile = fopen(FILE_NAME, "w");
  if (outfile) {
    genotype_fwrite(fittest, outfile);
    fclose(outfile);
    printf("wrote best genotype into %s\n", FILE_NAME);
  }
  else
    printf("unable to write %s\n", FILE_NAME);
  
  population_destroy(population);
}

int main(int argc, const char *argv[]) {
  // initialize Webots
  wb_robot_init();
  
  // get simulation step
  time_step = wb_robot_get_basic_time_step();

  // initialise emitter and receiver
  emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, get_time_step());

  // initialise the graph display
  display = wb_robot_get_device("display");
  display_width = wb_display_get_width(display);
  display_height = wb_display_get_height(display);
  wb_display_draw_text(display, "fitness", 2, 2);

  // initial population
  population = population_create(POPULATION_SIZE, GENOTYPE_SIZE);
  
  // find robot node and store initial position and orientation
  WbNodeRef robot = wb_supervisor_node_get_from_def("ROBOT");
  robot_translation = wb_supervisor_node_get_field(robot, "translation");
  robot_rotation = wb_supervisor_node_get_field(robot, "rotation");
  memcpy(robot_trans0, wb_supervisor_field_get_sf_vec3f(robot_translation), sizeof(robot_trans0));
  memcpy(robot_rot0, wb_supervisor_field_get_sf_rotation(robot_rotation), sizeof(robot_rot0));
  oldx = robot_rot0[X];
  oldy = robot_rot0[Y];
    
  // run GA optimization
  run_optimization();
  
  // cleanup Webots
  wb_robot_cleanup();
  return 0;  // ignored
}
