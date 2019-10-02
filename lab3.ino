#include <Sparki.h>

#define M_PI          3.14159
#define ROBOT_SPEED   0.0273
#define CYCLE_TIME    0.050
#define AXLE_DIAMETER 0.0857
#define WHEEL_RADIUS  0.03
#define THRESHOLD     0.01
#define HEADING_THRESHOLD 0.03
#define BUFFER        M_PI/4

#define CONTROLLER_STOP                0
#define CONTROLLER_GOTO_POSITION_PART2 1
#define CONTROLLER_GOTO_POSITION_PART3 2

#define THETA_MODE_BEARING 0
#define THETA_MODE_HEADING 1
const char* STATE_STRING[] = {
   "STOP",
   "PART2",
   "PART3"
};

int current_state = CONTROLLER_GOTO_POSITION_PART3;

// Odometry bookkeeping
double pose_x = 0, pose_y = 0, pose_theta = 0;
double dest_pose_x = 0, dest_pose_y = 0, dest_pose_theta = 0;

double d_err = 0, b_err = 0, h_err = 0;
double phi_l = 0, phi_r = 0;

// Wheel rotation vars
double left_speed_pct = 0;
double right_speed_pct = 0;

int left_dir = DIR_CCW;
int right_dir = DIR_CW;

bool flip_left = false;
bool flip_right = false;

double dX = 0, dTheta = 0;

bool at_target = false;

double to_radians(double deg) {
   return deg * 3.1415 / 180;
}

double to_degrees(double rad) {
   return rad * 180 / 3.1415;
}

void setup() {
   pose_x = 0;
   pose_y = 0;
   pose_theta = 0;
   set_pose_destination(0.15, -0.15, to_radians(150));
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(double x, double y, double t) {
   dest_pose_x = x;
   dest_pose_y = y;
   dest_pose_theta = t;
   if (dest_pose_theta > M_PI)  dest_pose_theta -= 2 * M_PI;
   if (dest_pose_theta < -M_PI) dest_pose_theta += 2 * M_PI;
}

void updateOdometry(char theta_mode) {
   d_err = sqrt(
       pow(pose_x - dest_pose_x, 2) +
       pow(pose_y - dest_pose_y, 2)
   );
   b_err = atan2(dest_pose_y - pose_y, dest_pose_x - pose_x) - pose_theta;
   h_err = dest_pose_theta - pose_theta;

   if (b_err > M_PI + BUFFER) {
      b_err -= 2 * M_PI;
   }
   if (b_err <= -M_PI - BUFFER) {
      b_err += 2 * M_PI;
   }

   if (h_err > M_PI + BUFFER) {
      h_err -= 2 * M_PI;
   }
   if (h_err <= -M_PI - BUFFER) {
      h_err += 2 * M_PI;
   }

   const double theta_err = (theta_mode == THETA_MODE_BEARING) ? b_err : h_err;

   if (theta_mode == THETA_MODE_HEADING) d_err = 0;

   phi_l = (d_err - ((theta_err * AXLE_DIAMETER) / 2)) / WHEEL_RADIUS;
   phi_r = (d_err + ((theta_err * AXLE_DIAMETER) / 2)) / WHEEL_RADIUS;

   if (phi_l > phi_r) {
       left_speed_pct  = 1;
       right_speed_pct = phi_r / phi_l;
   } else {
       left_speed_pct  = phi_l / phi_r;
       right_speed_pct = 1;
   }

   const double left_change  = left_speed_pct  * ROBOT_SPEED * CYCLE_TIME;
   const double right_change = right_speed_pct * ROBOT_SPEED * CYCLE_TIME;

   pose_theta += (right_change - left_change) / AXLE_DIAMETER;

   const double pose_vector = (right_change + left_change) / 2;

   pose_x += cos(pose_theta) * pose_vector;
   pose_y += sin(pose_theta) * pose_vector;

   // Bound theta.
   if (pose_theta >   M_PI) pose_theta -= 2 * M_PI;
   if (pose_theta <= -M_PI) pose_theta += 2 * M_PI;

   flip_left  = phi_l < 0;
   flip_right = phi_r < 0;
}

void displayOdometry() {
   sparki.print("State: ");
   sparki.println(STATE_STRING[current_state]);
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
   sparki.print(dX);
   sparki.print("   dT: ");
   sparki.println(dTheta);
   sparki.print("phl: ");
   sparki.print(phi_l);
   sparki.print(" phr: ");
   sparki.println(phi_r);
   sparki.print("p: ");
   sparki.print(d_err);
   sparki.print(" a: ");
   sparki.println(to_degrees(b_err));
   sparki.print("h: ");
   sparki.print(to_degrees(h_err));
}

void loop() {
   unsigned long begin_time = millis();
   unsigned long end_time = 0;
   unsigned long delay_time = 0;

   switch (current_state) {
       case CONTROLLER_GOTO_POSITION_PART2:
           updateOdometry(THETA_MODE_BEARING);

           // Correct bearing error.
           if (b_err > 0) {
               if (to_degrees(b_err) <= 180) {
                   sparki.moveRight(to_degrees(b_err));
               } else {
                   sparki.moveLeft(360 - to_degrees(b_err));
               }
               h_err -= b_err;
               b_err = 0;
           }

           // Correct distance error.
           if (d_err > 0) {
               sparki.moveForward(d_err * 100);
               d_err = 0;
           }

           // Correct heading error.
           if (h_err > 0) {
               sparki.moveRight(to_degrees(h_err));
               h_err = 0;
           }

           // Stop.
           sparki.moveStop();
           current_state = CONTROLLER_STOP;

           break;
       case CONTROLLER_GOTO_POSITION_PART3:
           // Theta mode is bearing when far from the target, and heading when
           // close to the target.
           updateOdometry(
               at_target ? THETA_MODE_HEADING : THETA_MODE_BEARING
           );

           if (d_err < THRESHOLD) {
              at_target = true;
           }

           if (at_target && abs(h_err) < HEADING_THRESHOLD) {
               // We are at the destination.
               sparki.moveStop();
               current_state = CONTROLLER_STOP;
               break;
           }

           sparki.motorRotate(
               MOTOR_LEFT,
               flip_left ? right_dir : left_dir,
               abs(int(left_speed_pct * 100))
           );
           sparki.motorRotate(
               MOTOR_RIGHT,
               flip_right ? left_dir : right_dir,
               abs(int(right_speed_pct * 100))
           );

           break;
   }

   sparki.clearLCD();
   displayOdometry();
   sparki.updateLCD();

   end_time = millis();
   delay_time = end_time - begin_time;
   delay(
       (delay_time < 1000 * CYCLE_TIME) ? 1000 * CYCLE_TIME - delay_time : 10
   );
}










