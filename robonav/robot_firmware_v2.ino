#include <micro_ros_arduino.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include <rmw_microros/rmw_microros.h>
#include <Encoder.h>

#include <sensor_msgs/msg/imu.h>
#include <Adafruit_Sensor_Calibration.h>

#include <Adafruit_AHRS.h>
#include <imuFilter.h>
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

//////////////// pin setup /////////////////

#define LED_PIN 13
#define r_dir1_pin 2
#define r_dir2_pin 0
#define r_pwm_pin 1

#define l_dir1_pin 3
#define l_dir2_pin 4
#define l_pwm_pin 5

#define l_encoder_a 7
#define l_encoder_b 8

#define r_encoder_a 9
#define r_encoder_b 10


////////////////Variables/////////////////////

rcl_publisher_t left_tick_pub;
std_msgs__msg__Int32 left_tick;

rcl_publisher_t right_tick_pub;
std_msgs__msg__Int32 right_tick;

rcl_publisher_t feedback_vel_pub;
geometry_msgs__msg__Twist feedback_vel_msg;

rcl_subscription_t wr_command_sub;
std_msgs__msg__Float32 wr_command;

rcl_subscription_t wl_command_sub;
std_msgs__msg__Float32 wl_command;

rcl_subscription_t pid_gain_sub;
geometry_msgs__msg__Vector3 pid_gain;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

bool micro_ros_init_successful;

unsigned int SPIN_FREQ = 20;
float windup_guard = 20;

// PID control parameters
float kp = 1.8;
float ki = 0.02;
float kd = 0.005;

//Motor pulse parameters
float PPR = 700 * 4;
long pl, pr;

//Left Wheel control parameter
unsigned long curTimeL, prevTimeL, dtL;
long curTickL, prevTickL, diffTickL;
double errL, prev_errL, sumErrL, dErrL, setRPML;
double control_outL;
double measuredRPML;
double desiredRPMR, desiredRPML;

/*Right Wheel Control*/
unsigned long curTimeR, prevTimeR, dtR;
long curTickR, prevTickR, diffTickR;
double errR, prev_errR, sumErrR, dErrR, setRPMR;
double control_outR;
double measuredRPMR;

int last_dirl;
int last_dirr;

//===============IMU ====================
float ax, ay, az, gx, gy, gz;
float ax_err, ay_err, az_err, gx_err, gy_err, gz_err;
float ax_cal, ay_cal, az_cal, gx_cal, gy_cal, gz_cal;
float qw, qx ,qy , qz;
float yaw;
// Sensor fusion
constexpr float GAIN = 0.1;     // Fusion gain, value between 0 and 1 - Determines response of heading correction with respect to gravity.
imuFilter <&GAIN> fusion;


//Find max value function
float max(float num1, float num2)
{
  return (num1 > num2) ? num1 : num2;
}

//Find min value function
float min(float num1, float num2)
{
  return (num1 > num2) ? num2 : num1;
}

void publishIMU(){
  getIMU();
  getQuaternion();

  imu_msg.orientation.w = qw;
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;

  imu_msg.linear_acceleration.x = ax_cal*9.81;
  imu_msg.linear_acceleration.y = ay_cal*9.81;
  imu_msg.linear_acceleration.z = az_cal*9.81;

  imu_msg.angular_velocity.x = gx_cal;
  imu_msg.angular_velocity.y = gy_cal;
  imu_msg.angular_velocity.z = gz_cal;

  imu_msg.angular_velocity_covariance[0] = 0.0001;
  imu_msg.angular_velocity_covariance[4] = 0.0001;
  imu_msg.angular_velocity_covariance[8] = 0.0001;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[8] = 0.0025;

  rcl_publish(&imu_pub, &imu_msg, NULL);
  
  
}


void getQuaternion() {
  fusion.update( gx_cal, gy_cal, gz_cal, ax_cal, ay_cal, az_cal );
  float v[3] = { 1, 1, 0 };
  float x[3], y[3], z[3];
  
  // Unit vectors of rectangular coodinates
#define TO_WORLD false
  // Project local axis to global reference frame = true
  // Project global axis to local reference frame = false

  fusion.getXaxis( TO_WORLD, x );
  fusion.getYaxis( TO_WORLD, y );
  fusion.getZaxis( TO_WORLD, z );
  fusion.projectVector( TO_WORLD, v );
  float q[4];
  fusion.getQuat(q);
  qw = q[0];
  qx = q[1];
  qy = q[2];
  qz = q[3];

}


bool init_sensors(void) {
  if (!fxos.begin() || !fxas.begin()) {
    return false;
  }
  accelerometer = fxos.getAccelerometerSensor();
  gyroscope = &fxas;
  magnetometer = fxos.getMagnetometerSensor();

  return true;
}


void imu_setup() {
  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  //setup_sensors();

  Wire.setClock(1000000); // 400KHz

}


void getIMURaw() {
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);
  ax = accel.acceleration.x / 9.81 ; //m/s^2 *0.1019
  ay = accel.acceleration.y / 9.81 ;
  az = accel.acceleration.z / 9.81 ;
  gx = gyro.gyro.x;   // rad/s
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;

}

void calibrateIMU(){
  int c =0;
  while(c < 3000){
    getIMURaw();
    gx_err += gx;
    gy_err += gy;
    gz_err += gz;
    ax_err += ax;
    ay_err += ay;
    az_err += az;
    c++;
  }
  ax_err = ax_err/c;
  ay_err = ay_err/c;
  az_err = (az_err/c)-1.0;  //x 0.1  //y 0.05  //z 1.0
  gx_err = gx_err/c;
  gy_err = gy_err/c;
  gz_err = gz_err/c;
}

void getIMU(){
  getIMURaw();
  ax_cal = ax - ax_err;
  ay_cal = ay - ay_err;
  az_cal = az - az_err;
  gx_cal = gx - gx_err;
  gy_cal = gy - gy_err;
  gz_cal = gz - gz_err;
}

//Left Motor Controller Function
void computePIDL(double control_cmd,
                 long inTick)
{
  int dirs = 1;  //1 for forward direction
  //int dirm = 1;  //1 for forward direction

  curTickL = inTick;
  curTimeL = millis();

  setRPML = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTickL = curTickL - prevTickL;
  dtL =  (curTimeL - prevTimeL);

  measuredRPML = ((diffTickL / PPR) / (dtL * 0.001)) * 60;

  errL = abs(setRPML) - abs(measuredRPML);
  if (dirs != last_dirl) {
    sumErrL = 0;
  }

  sumErrL += errL * dtL;

  dErrL = (errL - prev_errL) / dtL;

  control_outL = kp * errL + ki * sumErrL + kd * dErrL;

  if (control_outL < 0) {
    control_outL = 0;
  }
  prev_errL = errL;
  prevTickL = curTickL;
  prevTimeL = curTimeL;

  if (dirs > 0.5)
  {
    digitalWrite(l_dir1_pin, HIGH);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, control_outL);
  }
  else if (dirs < -0.5)
  {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, HIGH);
    analogWrite(l_pwm_pin, control_outL);

  }
  else
  {
    digitalWrite(l_dir1_pin, LOW);
    digitalWrite(l_dir2_pin, LOW);
    analogWrite(l_pwm_pin, 0);
  }

  last_dirl = dirs;

}

//Right Motor Controller Function
void computePIDR(double control_cmd, long inTick)
{
  int dirs = 1;
  //int dirm = 1;

  curTickR = inTick;
  curTimeR = millis();

  setRPMR = control_cmd;
  dirs = control_cmd / abs(control_cmd);

  diffTickR = curTickR - prevTickR;
  dtR =  (curTimeR - prevTimeR);

  measuredRPMR = ((diffTickR / PPR) / (dtR * 0.001)) * 60;

  errR = abs(setRPMR) - abs(measuredRPMR);

  if (dirs != last_dirr) {
    sumErrR = 0;
  }

  sumErrR += errR * dtR;

  dErrR = (errR - prev_errR) / dtR;


  control_outR = kp * errR + ki * sumErrR + kd * dErrR;

  if (control_outR < 0) {
    control_outR = 0;
  }
  prev_errR = errR;
  prevTickR = curTickR;
  prevTimeR = curTimeR;



  if (dirs > 0.5)
  {
    digitalWrite(r_dir1_pin, HIGH);
    digitalWrite(r_dir2_pin, LOW);
    analogWrite(r_pwm_pin, control_outR);
  }
  else if (dirs < -0.5)
  {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, HIGH);
    analogWrite(r_pwm_pin, control_outR);

  }
  else
  {
    digitalWrite(r_dir1_pin, LOW);
    digitalWrite(r_dir2_pin, LOW);
    analogWrite(r_pwm_pin, 0);
  }

  last_dirr = dirs;

}

void pid_gain_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3 * gain_value = (const geometry_msgs__msg__Vector3 *)msgin;

  //Uncomment for tuning
  /*kp = gain_value->x;

    ki = gain_value->y;
    kd = gain_value->z;*/
}

//Right wheel command callback function
void rightwheel_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPMR = power->data;

}


//Left Wheel Command Callback Function
void leftwheel_callback(const void * msgin)
{
  const std_msgs__msg__Float32 * power = (const std_msgs__msg__Float32 *)msgin;

  desiredRPML = power->data;

}


//Timer callback function
void timer_callback(rcl_timer_t *timer,int64_t last_call_time)
{
  publishIMU();
  rcl_publish(&feedback_vel_pub, &feedback_vel_msg, NULL);
  RCLC_UNUSED(timer);
}

//Create entities function
bool create_entities()
{
  
  allocator = rcl_get_default_allocator();


  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "base_control_node", "", &support);

  rclc_subscription_init_default(
    &pid_gain_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "pid_gain");

  rclc_subscription_init_default(
    &wl_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "wheel_command_left");

  rclc_subscription_init_default(
    &wr_command_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "wheel_command_right");

  rclc_publisher_init_default(
    &left_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_tick");

  rclc_publisher_init_default(
    &right_tick_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_tick");

  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/raw");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(25),
    timer_callback);

  rclc_publisher_init_default(
    &feedback_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "feedback_vel");


  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &pid_gain_sub, &pid_gain, &pid_gain_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &wr_command_sub, &wr_command, &rightwheel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &wl_command_sub, &wl_command, &leftwheel_callback, ON_NEW_DATA);

  micro_ros_init_successful = true;
}

void destroy_entities()
{
  rcl_publisher_fini(&left_tick_pub, &node);
  rcl_publisher_fini(&right_tick_pub, &node);
  rcl_publisher_fini(&feedback_vel_pub, &node);
  rcl_publisher_fini(&imu_pub, &node);
  rcl_subscription_fini(&wr_command_sub, &node);
  rcl_subscription_fini(&wl_command_sub, &node);
  rcl_subscription_fini(&pid_gain_sub, &node);
  
  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //PWM pin setup
  pinMode(r_dir1_pin, OUTPUT);
  pinMode(r_dir2_pin, OUTPUT);
  pinMode(r_pwm_pin, OUTPUT);
  pinMode(l_dir1_pin, OUTPUT);
  pinMode(l_dir2_pin, OUTPUT);
  pinMode(l_pwm_pin, OUTPUT);
  
  imu_setup();
  calibrateIMU();
  micro_ros_init_successful = false;

}

Encoder r_encoder(r_encoder_a, r_encoder_b);
Encoder l_encoder(l_encoder_a, l_encoder_b);
long old_pl = -999;
long old_pr = -999;

long curRPM_time;
long prevRPM_time;
long curPl;
long curPr;
long prevPl;
long prevPr;
long diffRPM_time;
float RPM_l, RPM_r;

void counter_RPM(long inPl, long inPr) {
  
  curPl = inPl;
  curPr = inPr;
  curRPM_time = millis();
  diffRPM_time = curRPM_time - prevRPM_time ;
  RPM_l = (((curPl - prevPl) / PPR) / (diffRPM_time * 0.001)) * 60;
  RPM_r = (((curPr - prevPr) / PPR) / (diffRPM_time * 0.001)) * 60;


  prevPl = curPl;
  prevPr = curPr;
  prevRPM_time = curRPM_time;
  feedback_vel_msg.linear.x = RPM_l; // RPM_l;
  feedback_vel_msg.linear.y = RPM_r;
  feedback_vel_msg.angular.x = control_outL;
  feedback_vel_msg.angular.y = control_outR;
  feedback_vel_msg.angular.z = yaw;

}

void counter_tick()
{
  pl = l_encoder.read();

  if (pl != old_pl) {
    old_pl = pl;
    Serial.println(pl);
  }

  pr = r_encoder.read();

  if (pr != old_pr) {
    old_pr = pr;
  }

  left_tick.data = pl;
  right_tick.data = pr;

  rcl_publish(&right_tick_pub, &right_tick, NULL);
  rcl_publish(&left_tick_pub, &left_tick, NULL);

}


void loop()
{

  uint32_t delay_rec = 100000;
  if (RMW_RET_OK == rmw_uros_ping_agent(50, 2))
  {
    delay_rec = 5000;
    if (!micro_ros_init_successful) {
      create_entities();
      getIMU();
      fusion.setup( ax_cal, ay_cal, az_cal );     
    } else {
      counter_tick();

      computePIDR(desiredRPMR, pr);
      computePIDL(desiredRPML, pl);
      counter_RPM(pl, pr);

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
  }
  else if (micro_ros_init_successful)
  {
    destroy_entities();
  }

  delayMicroseconds(delay_rec);
}
