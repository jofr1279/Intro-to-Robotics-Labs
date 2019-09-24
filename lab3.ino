#include <Sparki.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0274 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_STOP 0 
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
int current_state = CONTROLLER_GOTO_POSITION_PART2;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)

// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(135));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void updateOdometry() {
  d_err = sqrt(pow((pose_x - dest_pose_x), 2) + pow((pose_y - dest_pose_y), 2));
  b_err = atan2(dest_pose_y - pose_y, dest_pose_x - pose_x) - pose_theta;
  h_err = dest_pose_theta - pose_theta;

  phi_l = (d_err - ((b_err * AXLE_DIAMETER) / 2)) / WHEEL_RADIUS;
  phi_r = (d_err + ((b_err * AXLE_DIAMETER) / 2)) / WHEEL_RADIUS;

  pose_x = ((phi_r * WHEEL_RADIUS) / 2) + ((phi_l * WHEEL_RADIUS) / 2);
  pose_y = 0;
  pose_theta = ((phi_r * WHEEL_RADIUS) / AXLE_DIAMETER) - ((phi_l * WHEEL_RADIUS) / AXLE_DIAMETER);
  
  // Bound theta
  if (pose_theta > M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -M_PI) pose_theta += 2.*M_PI;
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        // TODO: Fill in odometry code
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow
// (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      updateOdometry(); //update error
      // b_err and h_err are in radians, convert to degrees to pass into moveLeft moveRight
      // b_err is the amount of degrees that the goal position is to the left of the robot
      if (b_err > 0) {
        if (to_degrees(b_err) > 180) {
          sparki.moveRight(360 - to_degrees(b_err));
        } else {
          sparki.moveLeft(to_degrees(b_err));
        }
        b_err = 0;
      }
      else if (d_err > 0) {
        sparki.moveForward(d_err * 100);
        d_err = 0;
   }
      else if (h_err > 0) {
     sparki.moveLeft(to_degrees(h_err));
        h_err = 0;
      }
  else {
    sparki.moveStop();
  current_state = CONTROLLER_STOP;
}
      
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:      
      updateOdometry();
      //calculate d_err, b_err, h_err
      // TODO: Implement solution using motorRotate and proportional feedback controller.
      // sparki.motorRotate function calls for reference:
      //      sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
      //      sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

   if (phi_l > phi_r) {
     left_speed_pct = 1;
   right_speed_pct = phi_r/phi_l;
   } else {
     left_speed_pct = phi_l/phi_r;
   right_speed_pct = 1;
 }
   
 sparki.motorRotate(MOTOR_LEFT, left_dir, int(left_speed_pct*100.));
 sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct*100.));

      break;
  }

  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}



