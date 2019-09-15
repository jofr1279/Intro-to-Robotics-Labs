#include <Sparki.h>

#define USE_CLOSURE 0

#define STATE_CALIBRATE 0
#define STATE_FOLLOW_LINE 1
#define STATE_DONE 2

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 2
#define DIRECTION_LEFT 3
#define DIRECTION_RIGHT 4

#define LINE_NONE 0
#define LINE_LEFT 1
#define LINE_RIGHT 2
#define LINE_CENTER 3
#define LINE_FULL 4

#define THRESHOLD 700
#define LAP_COOLDOWN_MAX 5

const char* STATE_STRING[] = {
    "Calibrate",
    "Follow Line",
    "Done",
};

char current_state = STATE_CALIBRATE;
char current_direction = DIRECTION_STOP;

int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

struct pose {
    double x = 0;
    double y = 0;
    double angle = 0;
} current_pose;

struct pose recorded_poses[3];

double speed;
double angular_speed;
int current_lap = -1;
int lapCooldown = 0;

void readSensors() {
    line_left = sparki.lineLeft();
    line_right = sparki.lineRight();
    line_center = sparki.lineCenter();
}

char getLineStatus() {
    readSensors();

    const int left = line_left <= THRESHOLD;
    const int center = line_center <= THRESHOLD;
    const int right = line_right <= THRESHOLD;

    if (center) {
        if (left && right) return LINE_FULL;
        return LINE_CENTER;
    }

    if (left) return LINE_LEFT;
    if (right) return LINE_RIGHT;

    return LINE_NONE;
}

double measureSpeed() {
    unsigned long travelTimeStart = millis();

    sparki.moveForward(30);

    unsigned long travelTimeEnd = millis();

    // We can assume travelTimeStart >= travelTimeEnd.
    double travelTimeDifference = (travelTimeEnd - travelTimeStart) / 1000.0;

    return 30 / travelTimeDifference;
}

double measureRotationalSpeed() {
    unsigned long travelTimeStart = millis();

    sparki.moveRight(180);

    unsigned long travelTimeEnd = millis();

    // We can assume travelTimeStart >= travelTimeEnd.
    double travelTimeDifference = (travelTimeEnd - travelTimeStart) / 1000.0;

    return PI / travelTimeDifference;
}


void updateOdometry() {
    // Speed and angular speed are in cm/s and rad/s respectively, and since the
    // loop is called every 100ms, divide by 10 to get the distance.
    double moved_dist = speed / 10;
    double rotated_dist = angular_speed / 10;

    switch (current_direction) {
        case DIRECTION_FORWARD:
            current_pose.x += moved_dist * cos(current_pose.angle);
            current_pose.y += moved_dist * sin(current_pose.angle);
            break;
        case DIRECTION_BACKWARD:
            current_pose.x -= moved_dist * cos(current_pose.angle);
            current_pose.y -= moved_dist * sin(current_pose.angle);
            break;
        case DIRECTION_LEFT:
            current_pose.angle -= rotated_dist;
            break;
        case DIRECTION_RIGHT:
            current_pose.angle += rotated_dist;
            break;
    }
}

void displayOdometry() {
    sparki.print("State: ");
    sparki.println(STATE_STRING[current_state]);

    sparki.print("Speed: ");
    sparki.println(speed);

    sparki.print("Angular Speed: ");
    sparki.println(angular_speed);

    sparki.print("Pose: ");
    sparki.print(current_pose.x);
    sparki.print(", ");
    sparki.println(current_pose.y);
    sparki.print("Angle: ");
    sparki.println(current_pose.angle);

    sparki.print("Lap: ");
    sparki.print(current_lap + 1);
    sparki.println("/3");
}

// Helper function that sets the current direction and also sets the robot's
// direction.
void setDirection(char direction) {
    switch (direction) {
        case DIRECTION_STOP:
            sparki.moveStop();
            break;
        case DIRECTION_FORWARD:
            sparki.moveForward();
            break;
        case DIRECTION_BACKWARD:
            sparki.moveBackward();
            break;
        case DIRECTION_LEFT:
            sparki.moveLeft();
            break;
        case DIRECTION_RIGHT:
            sparki.moveRight();
            break;
    }

    current_direction = direction;
}

void setup() {}

void loop() {
    unsigned long loopStartTime = millis();

    sparki.clearLCD();

    switch (current_state) {
        case STATE_CALIBRATE:
            sparki.println("Calibrating...");
            sparki.updateLCD();

            // Calibrate sparki (set the speeds).
            speed = measureSpeed();
            angular_speed = measureRotationalSpeed();

            // The calibration is complete.
            // Instruct the user to place Sparki before the start line.
            sparki.clearLCD();
            sparki.println("Calibration complete.");
            sparki.println("");
            sparki.println("Place before");
            sparki.println("start line.");
            sparki.updateLCD();
            delay(5000);

            current_state = STATE_FOLLOW_LINE;
            break;
        case STATE_FOLLOW_LINE:
            if (lapCooldown) {
                lapCooldown--;

                // Sometimes sparki will turn randomly when near the
                // line, so don't let him during lap cooldown.
                setDirection(DIRECTION_FORWARD);
            }

            // Follow the line.
            else switch (getLineStatus()) {
                case LINE_LEFT:
                    // We are either drifting towards the left, or a right turn
                    // has come up. Either way, turn right.
                    setDirection(DIRECTION_LEFT);
                    break;
                case LINE_RIGHT:
                    // We are either drifting towards the right, or a left turn
                    // has come up. Either way, turn left.
                    setDirection(DIRECTION_RIGHT);
                    break;
                case LINE_NONE:
                    // We have lost the line, default to moving forward for now.
                case LINE_CENTER:
                    // We are on track, keep moving forward!
                    setDirection(DIRECTION_FORWARD);
                    break;
                case LINE_FULL:
                    // Only increment the lap counter if the cooldown is up.
                    if (!lapCooldown) {
                        // Record the current pose.
                        if (current_lap >= 0) {
                            recorded_poses[current_lap].x = current_pose.x;
                            recorded_poses[current_lap].y = current_pose.y;
                            recorded_poses[current_lap].angle = current_pose.angle;
                        }

                        current_lap++;

                        // If all three laps have been taken, stop.
                        if (current_lap == 3) current_state = STATE_DONE;

                        // Reset the current pose.
                        if (USE_CLOSURE || !current_lap) {
                            current_pose.x = 0;
                            current_pose.y = 0;
                            current_pose.angle = 0;
                        }

                        // Use a cooldown so that the start line isn't
                        // registered twice in one lap.
                        lapCooldown = LAP_COOLDOWN_MAX;
                    }

                    break;
            }
            
            break;
        case STATE_DONE:
            sparki.moveStop();
        
            sparki.println("Results:");
            for (char i = 0; i < 3; ++i) {
                sparki.print(i + 1);
                sparki.print(": ");
                sparki.print(recorded_poses[i].x);
                sparki.print(", ");
                sparki.print(recorded_poses[i].y);
                sparki.print(", ");
                sparki.println(recorded_poses[i].angle);
            }
            break;
    }

    if (current_state != STATE_DONE) {
        updateOdometry();
        displayOdometry();
    }

    sparki.updateLCD();

    unsigned long loopEndTime = millis();

    // We can assume loopEndTime >= loopStartTime.
    unsigned long difference = loopEndTime - loopStartTime;

    // Check for integer underflow.
    if (difference > 100) difference = 100;

    delay(100 - difference);
}
