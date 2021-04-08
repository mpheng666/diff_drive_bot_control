#include <Arduino.h>
#include <robot_params.h>
#include <math.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#define p_gain 1.0
#define i_gain 0.000
#define d_gain 0.000
#define motor_min_rpm 0.0
#define motor_max_rpm 105.0

ros::NodeHandle nh;
geometry_msgs::TransformStamped tf_msg;
nav_msgs::Odometry odom_msg;
std_msgs::Float64MultiArray feedback_msg;
ros::Publisher odom_pub("/odom", &odom_msg);
ros::Publisher feedback_pub("/feedback", &feedback_msg);
tf::TransformBroadcaster broadcaster;

long rpm_R_measured;
long rpm_L_measured;
long vel_R;
long vel_L;
volatile int ENCO_R_count = 0;
volatile int ENCO_L_count = 0;
long previousMillis;
long currentMillis;
long interval = 10.0;
long rpm_R_desired;
long pwm_R_desired;
long rpm_L_desired;
long pwm_L_desired;

//PID variables
long error_R = 0;
long integral_R = 0;
long derivative_R = 0;
long error_prev_R = 0;
long integral_prev_R = 0;
long pwm_R_factor = 0;
long pwm_R_input = 0;
long error_L = 0;
long integral_L = 0;
long derivative_L = 0;
long error_prev_L = 0;
long integral_prev_L = 0;
long pwm_L_factor = 0;
long pwm_L_input = 0;
long pwm_factor_max = 100.0;

float linear_vel;
float angular_vel;
float distance_R_per_loop;
float distance_L_per_loop;
float distance_average;
float distance_total;

float x;
float y;
float theta;

char base_link[] = "/base_link";
char odom[] = "/wheel_odom";

void updateEncoder_R_1()
{
  if (read_R_1 != read_R_2)
  {
    ENCO_R_count++;
  }
  else
  {
    ENCO_R_count--;
  }
}

void updateEncoder_R_2()
{
  if (read_R_1 == read_R_2)
  {
    ENCO_R_count++;
  }
  else
  {
    ENCO_R_count--;
  }
}

void updateEncoder_L_1()
{
  if (read_L_2 != read_L_1)
  {
    ENCO_L_count++;
  }
  else
  {
    ENCO_L_count--;
  }
}

void updateEncoder_L_2()
{
  if (read_L_1 == read_L_2)
  {
    ENCO_L_count++;
  }
  else
  {
    ENCO_L_count--;
  }
}

void vel_sub_callback(const geometry_msgs::Twist &vel_msg)
{
  linear_vel = vel_msg.linear.x;
  angular_vel = vel_msg.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("/cmd_vel", vel_sub_callback);

void setup()
{
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(feedback_pub);
  broadcaster.init(nh);
  delay(3000);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(ENCO_R_1, INPUT_PULLUP);
  pinMode(ENCO_R_2, INPUT_PULLUP);
  pinMode(ENCO_L_1, INPUT_PULLUP);
  pinMode(ENCO_L_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCO_R_1), updateEncoder_R_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCO_R_2), updateEncoder_R_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCO_L_1), updateEncoder_L_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCO_L_2), updateEncoder_L_2, CHANGE);
  previousMillis = millis();
}

void loop()
{
  nh.spinOnce();
  currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;

    // for debugging
    // linear_vel = 0.0;
    // angular_vel = 0.5;

    // compute velocity of each wheel based on given twist
    vel_R = (linear_vel + (angular_vel * robot_width / 2.0)) / wheel_radius;
    vel_L = (linear_vel - (angular_vel * robot_width / 2.0)) / wheel_radius;

    // covert rad/s to rpm
    rpm_R_desired = vel_R / (2.0 * PI) * 60.0;
    rpm_L_desired = vel_L / (2.0 * PI) * 60.0;

    // convert rpm to pwm
    pwm_R_desired = 255.0 - (motor_max_rpm - rpm_R_desired) / motor_max_rpm * 255.0;
    pwm_L_desired = 255.0 - (motor_max_rpm - rpm_L_desired) / motor_max_rpm * 255.0;

    // PID speed control
    error_R = rpm_R_desired - rpm_R_measured;
    integral_R += error_R * interval;
    derivative_R = (error_R - error_prev_R) / interval;
    error_prev_R = error_R;
    integral_prev_R = integral_R;
    pwm_R_factor = error_R * p_gain + integral_R * i_gain + derivative_R * d_gain;

    error_L = rpm_L_desired - rpm_L_measured;
    integral_L += error_L * interval;
    derivative_L = (error_L - error_prev_L) / interval;
    error_prev_L = error_L;
    integral_prev_L = integral_L;
    pwm_L_factor = error_L * p_gain + integral_L * i_gain + derivative_L * d_gain;

    // limit pid gain to prevent motor from going crazy
    if (pwm_R_factor > pwm_factor_max)
    {
      pwm_R_input = pwm_R_desired;
    }
    else
    {
      pwm_R_input = pwm_R_factor + pwm_R_desired;
    }

    if (pwm_L_factor > pwm_factor_max)
    {
      pwm_L_input = pwm_L_desired;
    }
    else
    {
      pwm_L_input = pwm_L_factor + pwm_L_desired;
    }

    // write dir and pwm to pin
    if (pwm_R_input >= 0)
    {
      digitalWrite(DIR_R, 1);
      analogWrite(PWM_R, pwm_R_input);
    }
    else
    {
      digitalWrite(DIR_R, 0);
      analogWrite(PWM_R, abs(pwm_R_input));
    }

    if (pwm_L_input >= 0)
    {
      digitalWrite(DIR_L, 1);
      analogWrite(PWM_L, pwm_L_input);
    }
    else
    {
      digitalWrite(DIR_L, 0);
      analogWrite(PWM_L, abs(pwm_L_input));
    }

    // check count per loop
    // Serial.print("R count: ");
    // Serial.print(ENCO_R_count);
    
    // Serial.print("  L count: ");
    // Serial.println(ENCO_L_count);

    // convert encoder to rpm
    rpm_R_measured = ENCO_R_count * 6000.0 / (ENCO_CPR * GEAR_RATIO);
    rpm_L_measured = ENCO_L_count * 6000.0 / (ENCO_CPR * GEAR_RATIO);

    // convert encoder to distance travelled
    distance_R_per_loop = (ENCO_R_count / (ENCO_CPR * GEAR_RATIO)) * (2.0 * PI * wheel_radius);
    distance_L_per_loop = (ENCO_L_count / (ENCO_CPR * GEAR_RATIO)) * (2.0 * PI * wheel_radius);

    // compute distance travalled as a robot
    distance_average = (distance_R_per_loop + distance_L_per_loop) / 2.0;
    distance_total += distance_average;

    // compute theta
    theta += (distance_R_per_loop - distance_L_per_loop) / robot_width;

    // normalise theta
    if (theta > PI)
      theta -= TWO_PI;
    if (theta < -(PI))
      theta += TWO_PI;

    // compute x and y
    x += distance_average * cos(theta);;
    y += distance_average * sin(theta);

    // pub tf
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.frame_id = odom;
    tf_msg.child_frame_id = base_link;

    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation = tf::createQuaternionFromYaw(theta);
    tf_msg.header.stamp = nh.now();

    broadcaster.sendTransform(tf_msg);

    // pub odom
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = nh.now();
    odom_msg.header.frame_id = odom;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);

    odom_msg.child_frame_id = base_link;
    odom_msg.twist.twist.linear.x = distance_average * 100.0;
    odom_msg.twist.twist.linear.y = 0.0;     
    odom_msg.twist.twist.angular.z = (distance_R_per_loop - distance_L_per_loop) / robot_width * 100.0; 

    odom_pub.publish(&odom_msg);

    // for PID tunning
    // Serial.print("ERROR: ");
    // Serial.print(error_R);
    // Serial.print('\t');
    // Serial.print("PWM_FACTOR: ");
    // Serial.print(pwm_R_factor);
    // Serial.print('\t');
    // Serial.print("Desired PWM: ");
    // Serial.print(pwm_R_desired);
    // Serial.print('\t');
    // Serial.print(" ADJUSTED PWM: ");
    // Serial.print(pwm_R_input);
    // Serial.print('\t');
    // Serial.print("DESIRED SPEED: ");
    // Serial.print(rpm_R_desired);
    // Serial.println(" RPM");
    // Serial.print('\t');
    // Serial.print("ACTUAL SPEED: ");
    // Serial.print(rpm_R_measured);
    // Serial.println(" RPM");

    // reset encoder
    ENCO_R_count = 0;
    ENCO_L_count = 0;
  }
}