#include "nav.h"

// Initialize the position struct
struct position pos = {
    {{0}}, {{0}},             // Maps
    0.0, 0.0, 0.0,            // Roll, pitch, yaw
    {0.0, 0.0, 0.0},          // Accel
    0.0, 0.0, 0.0,            // Distances
    SEARCHING                 // Initial State
};

// Navigation/Pixy Tracking Variables
unsigned long tagTimeout = 0;
unsigned long tagInterval = 5000;
unsigned long ignoreCorrectionsUntil = 0;
float targetTagTurn = 0;
float targetTagError = 0;

const int CONFIRM_THRESHOLD = 5;
const int MIN_BLOCK_WIDTH = 25;
int lastSignature = -1;
int confirmCount = 0;


void updatePosition(float r, float p, float y, float ax, float ay, float az) {
    // 1. Sync distance sensors from drivers
    pos.D_L = D_L;
    pos.D_R = D_R;
    pos.D = D;
    
    // 2. Sync IMU orientation & acceleration
    pos.roll = r;
    pos.pitch = p;
    pos.yaw = y;
    pos.accel[0] = ax;
    pos.accel[1] = ay;
    pos.accel[2] = az;
}


void navigate() {
    // --- 1. EVALUATE STATE TRANSITIONS ---
    if (pos.currentState != TURN_TAG && pos.currentState != REVERSING) {
        bool tagConfirmedThisFrame = false;
        bool tagSeenInDistance = false;

        bool isTooCloseToWall = (pos.D_L < MIN_TURN_CLEARANCE || pos.D_R < MIN_TURN_CLEARANCE);
        bool isSafeToTurn = !isTooCloseToWall;

        pixy.ccc.getBlocks();
        if (pixy.ccc.numBlocks > 0) {
            int sig = pixy.ccc.blocks[0].m_signature;
            int width = pixy.ccc.blocks[0].m_width;
            int x_pos = pixy.ccc.blocks[0].m_x;

            if (width > MIN_BLOCK_WIDTH) { 
                tagTimeout = millis() + tagInterval;
                if (sig == lastSignature) {
                    confirmCount++;
                } else {
                    lastSignature = sig;
                    confirmCount = 1;
                }

                if (confirmCount >= CONFIRM_THRESHOLD) {
                    if (pos.D <= 10) { 
                        if (isSafeToTurn) {
                            tagConfirmedThisFrame = true;
                            if (sig == 1) targetTagTurn = -90.0;
                            if (sig == 2) targetTagTurn = 180.0;
                            if (sig == 3) targetTagTurn = 100.0;
                            
                            pos.currentState = TURN_TAG;
                            confirmCount = 0;
                            lastSignature = -1;
                        } else {
                            tagSeenInDistance = true;
                            targetTagError = x_pos - 157.0;
                        }
                    } else {
                        tagSeenInDistance = true;
                        targetTagError = x_pos - 157.0;
                    }
                }
            }
        } else {
            confirmCount = 0;
            lastSignature = -1;
        }

        if (tagSeenInDistance && (pos.currentState != TURN_TAG)) {
            pos.currentState = APPROACH_TAG;
        }
        else if (!tagConfirmedThisFrame && pos.currentState != TURN_TAG) {
            if (pos.D < (S / 3)) {
                pos.currentState = (pos.D_L < pos.D_R) ? PIVOT_RIGHT_DIST : PIVOT_LEFT_DIST;
            } 
            else if (pos.D_L + pos.D_R <= WALL_DIST_TOO_BIG && abs(pos.D_L - pos.D_R) > C && millis() > ignoreCorrectionsUntil) {
                pos.currentState = CENTERING;
            } 
            else if (pos.D_L > S && pos.D_R > S) {
                pos.currentState = SEARCHING;
            } 
            else if (millis() > tagTimeout && (pos.D_L > S || pos.D_R > S)){
                if (pos.D_R > pos.D_L){
                    pos.currentState = PIVOT_RIGHT_DIST;
                }
                else{
                    pos.currentState = PIVOT_LEFT_DIST;
                }
            }
            else {
                pos.currentState = DRIVE_STRAIGHT;
            }
        }
    }

    // --- 2. EXECUTE CURRENT STATE ---
    switch (pos.currentState) {
        case TURN_TAG:
            motors.setSpeeds(0, 0); 
            delay(150); 
            turn_deg(targetTagTurn);
            delay(150); 
            drive_dist(1.0); 
            ignoreCorrectionsUntil = millis() + 500; 
            pos.currentState = SEARCHING; 
            break;

        case REVERSING:
            motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
            delay(1500); 
            motors.setSpeeds(0, 0);
            turn_deg((pos.D_L < pos.D_R) ? 30.0 : -30.0); 
            ignoreCorrectionsUntil = millis() + 1500; 
            pos.currentState = SEARCHING;
            break;

        case SEARCHING:
            motors.setSpeeds(0, 0); 
            break;

        case DRIVE_STRAIGHT:
            motors.setSpeeds(BASE_SPEED, BASE_SPEED);
            break;

        case CENTERING:
            {
                float error = pos.D_R - pos.D_L; 
                float kP = 8.0; 
                int centerBaseSpeed = BASE_SPEED * 5 / 4;
                int leftSpeed = constrain(centerBaseSpeed + (error * kP), 50, 255);
                int rightSpeed = constrain(centerBaseSpeed - (error * kP), 50, 255);
                motors.setM1Speed(leftSpeed);
                motors.setM2Speed(rightSpeed);
            }
            break;

        case APPROACH_TAG:
            {
                float kP_pixy = .5; 
                int leftSpeed = constrain(BASE_SPEED + (targetTagError * kP_pixy), 50, 255);
                int rightSpeed = constrain(BASE_SPEED - (targetTagError * kP_pixy), 50, 255);
                motors.setM1Speed(leftSpeed);
                motors.setM2Speed(rightSpeed);
            }
            break;

        case PIVOT_LEFT_DIST:
            motors.setM1Speed(-TURN_SPEED);
            motors.setM2Speed(TURN_SPEED);
            break;

        case PIVOT_RIGHT_DIST:
            motors.setM1Speed(TURN_SPEED);
            motors.setM2Speed(-TURN_SPEED);
            break;
    }
}