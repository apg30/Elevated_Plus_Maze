// Description:

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <webots/nodes.h>
#include <webots/led.h>
#include <webots/device.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
static int time_step;

// Device stuff for distance sensors
#define NUM_SENSORS 8
static WbDeviceTag distance_sensors[NUM_SENSORS];
static double distance_sensors_values[NUM_SENSORS];
static double distance_sensors_values_full[NUM_SENSORS];
static const char *distance_sensors_names[NUM_SENSORS] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

// Device stuff for ground sensors
#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = { 0.0, 0.0, 0.0 };
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {
  "gs0", "gs1", "gs2"
};

// Device stuff for emitter and receiver
static WbDeviceTag receiver;
static WbDeviceTag emitter;

// Device stuff for wheels
#define NUM_WHEELS 2
#define MAX_SPEED 1000.0
static double speeds[NUM_WHEELS];

// Genotype stuff
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
// Breitenberg sensor weights for object avoidance
static double matrix[NUM_SENSORS][NUM_WHEELS];

// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  
  if (wb_receiver_get_queue_length(receiver) > 0) {
  
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes into weight matrix
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
    
    // prepare for receiving next packet
    wb_receiver_next_packet(receiver);
  }
}

static double offsets[2] = {
  0.5*MAX_SPEED, 0.5*MAX_SPEED
};

// find simulation step
static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int) wb_robot_get_basic_time_step();
  return time_step;
}

// used in init_Devices()
static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Initialise sensors
static void init_devices() {
  int i;
  
  // Distance sensors
  for (i=0; i<NUM_SENSORS; i++) {
    distance_sensors[i]=wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  
  // Ground sensors
  for (i=0; i<GROUND_SENSORS_NUMBER; i++) {
    ground_sensors[i] = wb_robot_get_device(ground_sensors_names[i]);
    wb_distance_sensor_enable(ground_sensors[i], get_time_step());
  }
  
  // Emitter and receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, get_time_step());
  emitter = wb_robot_get_device("emitter");
  
  step();
}

// Reset motors to stationary
static void reset_actuator_values() {
  speeds[0] = 0;
  speeds[1] = 0;
}

// Get sensor data
static void get_sensor_input() {
  int i;
  
  // Distance sensors
  for (i=0; i<NUM_SENSORS; i++) {
  
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);
    distance_sensors_values_full[i] = distance_sensors_values[i];
    distance_sensors_values[i] /= 4096;
    // This scales the sensor data to a number between 0 (nothing) and 1 (wall)
  }

  // Ground sensors
  for (i=0; i<GROUND_SENSORS_NUMBER; i++)
    ground_sensors_values[i] = wb_distance_sensor_get_value(ground_sensors[i]);
}

// Set motors to their respective speeds
static void set_actuators() {
  wb_differential_wheels_set_speed(speeds[0], speeds[1]);
}

  // compute actuation using Braitenberg's algorithm:
  // multiply sensor weight by sensor value, then add to the motor speed
static void run_braitenberg() {
  int i, j;
  for (i=0; i<2; i++) {   //for each wheel
    speeds[i] = 0.0;      //set the speed to 0
    for (j=0; j<NUM_SENSORS; j++)  //for each sensor
    //set the speed to distance to an object multiplied by its weight.
      speeds[i] += distance_sensors_values[j] * matrix[j][i];
      speeds[i] = offsets[i] + speeds[i]*MAX_SPEED;
    
    //ensure the speed is not more than 1000 (prevents bugs)
    if (speeds[i] > MAX_SPEED)
      speeds[i] = MAX_SPEED;
    else if (speeds[i] < -MAX_SPEED)
      speeds[i] = -MAX_SPEED;
  }
}

// send sensor data through the emitter
static void senddata(){
  int e,f;
  static double sensorCombined[11];
  
  // first 8 array entries are IR sensor data
  for (e = 0; e < 8; e++)
    sensorCombined[e] = distance_sensors_values_full[e];
  
  // next 3 entries are ground sensor data
  for (f = 0; f < 3; f++)
    sensorCombined[f + 8] = ground_sensors_values[f];

  // send data
  wb_emitter_send(emitter, sensorCombined, sizeof(sensorCombined));

}
//************************************************************************

int main(int argc, const char *argv[]) {
  wb_robot_init();  // initialize Webots
  printf("Controller started....\n");
  init_devices();
  time_step = wb_robot_get_basic_time_step();
  
  // initialize matrix to zero, hence the robot wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));

  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {  
    check_for_new_genes();
    reset_actuator_values();
    get_sensor_input();
    senddata();
    run_braitenberg();
    set_actuators();
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}