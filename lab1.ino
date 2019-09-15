#include <Sparki.h>

#define STATE_SPIN_360 0
#define STATE_MOVE_TO_OBJECT 1
#define STATE_PICK_UP_OBJECT 2
#define STATE_SPIN_180 3
#define STATE_MOVE_TO_LINE 4
#define STATE_FOLLOW_LINE_TO_START 5
#define STATE_DROP_OBJECT 6
#define STATE_DONE 7

#define GRIPPER_OPEN 0
#define GRIPPER_CLOSE 1

#define LINE_NONE 0
#define LINE_LEFT 1
#define LINE_RIGHT 2
#define LINE_CENTER 3
#define LINE_FULL 4

#define THRESHOLD 700

const char* STATE_STRING[] = {
    "Spin 360",
    "Move To Object",
    "Pick Up Object",
    "Spin 180",
    "Move To Line",
    "Follow Line To Start",
    "Drop Object",
    "Done",
};

const char* LINE_STRING[] = {
    "None",
    "Left",
    "Right",
    "Center",
    "Full",
};

char current_state = STATE_SPIN_360;
int cmDistance;
int lineLeft;
int lineCenter;
int lineRight;
char lineStatus;
int correctionTurns = 1;
int correctionTurnProgress = 0;

void setup();
void readSensors();
void centerUltrasonicSensor();
void setGripper(char);
int objectInDistance(int);
char getLineStatus();
void printDebugInfo();
void loop();

void setup() {
    sparki.RGB(RGB_RED);

    centerUltrasonicSensor();
    setGripper(GRIPPER_OPEN);

    sparki.RGB(RGB_GREEN);
}

void readSensors() {
    cmDistance = sparki.ping();
    lineLeft = sparki.lineLeft();
    lineRight = sparki.lineRight();
    lineCenter = sparki.lineCenter();
    lineStatus = getLineStatus();
}

void centerUltrasonicSensor() {
    sparki.servo(SERVO_CENTER);

    // Give the servo time to move.
    delay(1000);
}

void setGripper(char state) {
    if (state == GRIPPER_OPEN) sparki.gripperOpen();
    else sparki.gripperClose();

    // Give the gripper time to move.
    delay(7000);

    sparki.gripperStop();
}

int objectInDistance(int distance) {
    return cmDistance != -1 && cmDistance <= distance;
}

char getLineStatus() {
    const int left = lineLeft < THRESHOLD;
    const int center = lineCenter < THRESHOLD;
    const int right = lineRight < THRESHOLD;

    if (center) {
        if (left && right) return LINE_FULL;
        return LINE_CENTER;
    }

    if (left) return LINE_LEFT;
    if (right) return LINE_RIGHT;

    return LINE_NONE;
}

void printDebugInfo() {
    // Print the state.
    sparki.print("State: ");
    sparki.println(STATE_STRING[current_state]);

    // Print the line information.
    sparki.print("Line: ");
    sparki.print(lineLeft);
    sparki.print(" ");
    sparki.print(lineCenter);
    sparki.print(" ");
    sparki.println(lineRight);
    sparki.print("Line Status: ");
    sparki.println(LINE_STRING[lineStatus]);

    // Print the distance information.
    sparki.print("Distance: ");
    sparki.println(cmDistance);

    // Print the correction turns counter.
    sparki.print("Correction Turn Status: ");
    sparki.print(correctionTurnProgress);
    sparki.print("/");
    sparki.println(correctionTurns);
}

void loop() {
    readSensors();

    sparki.clearLCD();
    printDebugInfo();

    switch (current_state) {
        case STATE_SPIN_360:
            // Spin to the right.
            sparki.moveRight();

            // If there is an object 30 cm ahead, turn left a bit to correct for
            // overshot, and start moving toward the object.
            if (objectInDistance(30)) {
                sparki.moveLeft(3);
                current_state = STATE_MOVE_TO_OBJECT;
            }

            break;
        case STATE_MOVE_TO_OBJECT:
            // Move forward.
            sparki.moveForward();

            // Check if the object has been lost.
            if (!objectInDistance(40)) {
                // Move left and right by increasing amounts until the object
                // comes back into view.
                if ((int)(log(correctionTurns) / log(2)) % 2)
                    sparki.moveLeft(5);
                else sparki.moveRight(5);

                // Keep track of how much we turned so we know when to change
                // direction.
                correctionTurnProgress += 5;

                // If we hit the end of a turn and still haven't found the
                // object, double the turn distance and reset the progress.
                if (correctionTurnProgress > correctionTurns) {
                    correctionTurns *= 2;
                    correctionTurnProgress = 0;
                }

                break;
            }

            // The object has been found, so reset the correction turn counter
            // and progress.
            correctionTurns = 1;
            correctionTurnProgress = 0;

            // Drive until the object is less than 8 cm away, move a little
            // further to make sure the object is in the grippers, and then
            // start picking up the object.
            if (objectInDistance(8)) {
                sparki.moveForward();
                delay(1000);
                current_state = STATE_PICK_UP_OBJECT;
            }

            break;
        case STATE_PICK_UP_OBJECT:
            // Stop moving for the rest of the state.
            sparki.moveStop();

            // Close the gripper around the object.
            setGripper(GRIPPER_CLOSE);

            // Back up a bit before turning.
            sparki.moveBackward();
            delay(1000);

            // Start spinning around.
            current_state = STATE_SPIN_180;

            break;
        case STATE_SPIN_180:
            // Turn around.
            sparki.moveRight(180);

            // Start moving to the line.
            current_state = STATE_MOVE_TO_LINE;

            break;
        case STATE_MOVE_TO_LINE:
            // Move forward for the rest of the state.
            sparki.moveForward();

            // Once the line is found, and start following the line.
            if (lineStatus != LINE_NONE)
                current_state = STATE_FOLLOW_LINE_TO_START;

            break;
        case STATE_FOLLOW_LINE_TO_START:
            // Follow the line.
            switch (lineStatus) {
                case LINE_LEFT:
                    // We are either drifting towards the left, or a right turn
                    // has come up. Either way, turn right.
                    sparki.moveLeft();
                    break;
                case LINE_RIGHT:
                    // We are either drifting towards the right, or a left turn
                    // has come up. Either way, turn left.
                    sparki.moveRight();
                    break;
                case LINE_NONE:
                    // We have lost the line, default to moving forward for now.
                case LINE_CENTER:
                    // We are on track, keep moving forward!
                    sparki.moveForward();
                    break;
                case LINE_FULL:
                    // The start line has been found, so start dropping the
                    // object.
                    current_state = STATE_DROP_OBJECT;
                    break;
            }

            break;
        case STATE_DROP_OBJECT:
            // Stop for the rest of the program.
            sparki.moveStop();

            // Drop the object.
            setGripper(GRIPPER_OPEN);

            sparki.beep();

            // Switch to a "done" state to end the program.
            current_state = STATE_DONE;

            break;
    }

    sparki.updateLCD();

    // Prevent the controller from running too fast.
    delay(10);
}
