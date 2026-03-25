#include "nav.h"

// Initialize the position struct
struct position pos = {
    {{0}}, {{0}},             // Maps
    0.0, 0.0, 0.0,            // Roll, pitch, yaw
    {0.0, 0.0, 0.0},          // Accel
    99.0, 99.0, 99.0,            // Distances
    SEARCHING                 // Initial State
};

// Navigation/Pixy Tracking Variables
unsigned long tagTimeout = 0;
unsigned long tagInterval = 5000;
unsigned long ignoreCorrectionsUntil = 0;
float targetTagTurn = 0;
float targetTagError = 0;

const int CONFIRM_THRESHOLD = 5;
const int MIN_BLOCK_WIDTH = 15;
int lastSignature = -1;
int confirmCount = 0;

// --- IMU & STATE VARIABLES ---
float targetHeading = 0.0; 
unsigned long reverseTimer = 0;
int reversePhase = 0; 

// --- PID VARIABLES FOR CENTERING ---
float integralIR = 0.0;
float prevErrorIR = 0.0;
float integralIMU = 0.0;
float prevErrorIMU = 0.0;
unsigned long lastTimePID = 0;

// Constants
float kP_IR = 2.0;  
float kI_IR = 0.1;  
float kD_IR = 1.5;  

float kP_IMU = 6.0; 
float kI_IMU = 0.05;
float kD_IMU = 0.5;

// ----------------------------
float kP_pixy = 0.5; //Used in Approach_tag state. ONLY proportional.


float kP_Turn = 3.0; //Proportional variable for P-control in turning

// Helper function to calculate heading error with 360-degree wrapping
float getHeadingError() {
    float error = targetHeading - pos.yaw;
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

// Helper to add degrees to target heading and maintain 0-360 wrap
void updateTargetHeading(float degreesToAdd) {
    targetHeading += degreesToAdd;
    if (targetHeading >= 360.0) targetHeading -= 360.0;
    if (targetHeading < 0.0) targetHeading += 360.0;
}

// void updatePosition(float r, float p, float y, float ax, float ay, float az) {
//     pos.D_L = D_L;
//     pos.D_R = D_R;
//     pos.D = D;
    
//     pos.roll = r;
//     pos.pitch = p;
//     pos.yaw = y;
//     pos.accel[0] = ax;
//     pos.accel[1] = ay;
//     pos.accel[2] = az;
// }

void navigate() {
    // --- 1. EVALUATE STATE TRANSITIONS ---
    // Only evaluate new transitions if we are not currently locked in a mandatory maneuver
    bool isEvaluating = (pos.currentState == SEARCHING || pos.currentState == DRIVE_STRAIGHT || 
                         pos.currentState == CENTERING || pos.currentState == APPROACH_TAG);

    if (isEvaluating) {
        
        bool tagConfirmedThisFrame = false;
        bool tagSeenInDistance = false;

        bool isTooCloseToWall = (pos.D_L < MIN_TURN_CLEARANCE || pos.D_R < MIN_TURN_CLEARANCE);
        bool isSafeToTurn = !isTooCloseToWall;

        pixy.ccc.getBlocks();
        Serial.println("Check pixy");
        if (pixy.ccc.numBlocks > 0) {
            int sig = pixy.ccc.blocks[0].m_signature;
            int width = pixy.ccc.blocks[0].m_width;
            int x_pos = pixy.ccc.blocks[0].m_x;

            if (width > MIN_BLOCK_WIDTH) { 
                tagTimeout = millis() + tagInterval;
                Serial.println("tag passes min size");
                
                if (sig == lastSignature) {
                    confirmCount++;
                } 
                
                else {
                    // Only reset if it's completely wrong, to allow for minor frame drops
                    if (confirmCount > 0) {
                        confirmCount--; // Degrade gracefully instead of snapping to 0
                    } else {
                        lastSignature = sig;
                        confirmCount = 1;
                    }
                }
                if (confirmCount >= CONFIRM_THRESHOLD) {
                    if (pos.D > 2.0 && pos.D <= 10) { 
                        if (isSafeToTurn) {
                            tagConfirmedThisFrame = true;
                            
                            // SET ABSOLUTE HEADING BEFORE EXECUTING TURN
                            if (sig == 1) updateTargetHeading(-90.0);
                            if (sig == 2) updateTargetHeading(180.0);
                            if (sig == 3) updateTargetHeading(90.0); 
                            
                            pos.currentState = TURN_TAG;
                            confirmCount = 0;
                            lastSignature = -1;
                        } else {
                            tagSeenInDistance = true;
                            targetTagError = x_pos - 157.0;
                        }
                    } else {
                        tagSeenInDistance = true;
                        Serial.println("Tag seen in distance");
                        targetTagError = x_pos - 157.0;
                    }
                }
            }
        } else {
            confirmCount = 0;
            lastSignature = -1;
        }

        if (tagSeenInDistance && !tagConfirmedThisFrame) {
            pos.currentState = APPROACH_TAG;
        }
        else if (!tagConfirmedThisFrame) {
            
            float hErr = getHeadingError();
            bool offCenter = (abs(pos.D_L - pos.D_R) > C);
            bool offHeading = (abs(hErr) > 6.0); 

            // Priority 1: Front Wall Avoidance
            if (pos.D < (S / 3)) {
                Serial.println("Wall avoidance");
                if (pos.D_L < pos.D_R) {
                    updateTargetHeading(-90.0); // Turn Right
                    pos.currentState = PIVOT_RIGHT_DIST;
                    
                } else {
                    updateTargetHeading(90.0);  // Turn Left
                    pos.currentState = PIVOT_LEFT_DIST;
                }
            } 
            // Priority 2: Centering & Heading Correction
            else if (pos.D_L + pos.D_R <= WALL_DIST_TOO_BIG && (offCenter || offHeading) && millis() > ignoreCorrectionsUntil) {
                pos.currentState = CENTERING;
            } 
            // Priority 3: Open space
            else if (pos.D_L > S && pos.D_R > S) {
                pos.currentState = SEARCHING;
            } 
            // Priority 4: Maze Quirk (Tag Timeout)
            // else if (millis() > tagTimeout && (pos.D_L > S || pos.D_R > S)){
            //     if (pos.D_R > pos.D_L){
            //         updateTargetHeading(-90.0);
            //         pos.currentState = PIVOT_RIGHT_DIST;
            //     } else {
            //         updateTargetHeading(90.0);
            //         pos.currentState = PIVOT_LEFT_DIST;
            //     }
            // }
            // Default
            else {
                pos.currentState = DRIVE_STRAIGHT;
            }
        }
    }

    // --- 2. EXECUTE CURRENT STATE ---
    switch (pos.currentState) {
        
        // Combine all absolute turns into one robust IMU tracking block
        case TURN_TAG:
        case PIVOT_LEFT_DIST:
        case PIVOT_RIGHT_DIST:
            {
                Serial.println("Pivor Right Dist");
                float hErr = getHeadingError();
                
                // If we are within 3 degrees of the target, the turn is complete
                if (abs(hErr) <= 3.0) {
                    motors.setSpeeds(0, 0);
                    ignoreCorrectionsUntil = millis() + 500; 
                    pos.currentState = SEARCHING; // Drop into searching to naturally drive straight
                    break;
                }

                // P-Controller for smooth turning
                
                int minTurnSpeed = 65; // Minimum PWM to overcome motor friction
                
                int turnSpeed = abs(hErr) * kP_Turn;
                turnSpeed = constrain(turnSpeed, minTurnSpeed, TURN_SPEED);

                // Assuming positive hErr means target is to the Left (Counter-Clockwise)
                // If this spins the wrong way, swap the positive and negative signs below.
                if (hErr > 0) {
                    motors.setM1Speed(-turnSpeed); 
                    motors.setM2Speed(turnSpeed);
                } else {
                    motors.setM1Speed(turnSpeed);
                    motors.setM2Speed(-turnSpeed);
                }
            }
            break;

        case REVERSING:
             Serial.println("Reversing");
            if (reversePhase == 0) {
                motors.setSpeeds(-BASE_SPEED, -BASE_SPEED);
                if (reverseTimer == 0) reverseTimer = millis() + 1500; 

                // Done backing up, set absolute angle to escape
                if (millis() > reverseTimer) {
                    reversePhase = 1; 
                    if (pos.D_L < pos.D_R) {
                        updateTargetHeading(-45.0); // Escape right
                    } else {
                        updateTargetHeading(45.0);  // Escape left
                    }
                }
            } 
            else if (reversePhase == 1) {
                // Use the exact same closed-loop logic to turn out of the corner
                float hErr = getHeadingError();
                if (abs(hErr) <= 3.0) {
                    motors.setSpeeds(0, 0);
                    ignoreCorrectionsUntil = millis() + 1500; 
                    pos.currentState = SEARCHING;
                    reversePhase = 0; 
                    reverseTimer = 0;
                } else {
                    int turnSpeed = TURN_SPEED; 
                    if (hErr > 0) {
                        motors.setSpeeds(-turnSpeed, turnSpeed);
                    } else {
                        motors.setSpeeds(turnSpeed, -turnSpeed);
                    }
                }
            }
            break;

        case SEARCHING:
            Serial.println("Searching");
            motors.setSpeeds(0, 0); 
            break;

        case DRIVE_STRAIGHT:
            {   Serial.println("Straight");
                // float kP_IMU = 3.5; 
                // float hError = getHeadingError();
                
                // int leftSpeed = constrain(BASE_SPEED - (hError * kP_IMU), 50, 255);
                // int rightSpeed = constrain(BASE_SPEED + (hError * kP_IMU), 50, 255);
                
                motors.setM1Speed(BASE_SPEED);
                motors.setM2Speed(BASE_SPEED);
            }
            break;

        case CENTERING:
            {   Serial.println("Centering");
                float errorIR = pos.D_R - pos.D_L; 
                float errorIMU = getHeadingError();

                // PID Time Calculation
                unsigned long now = millis();
                float dt = (now - lastTimePID) / 1000.0;
                if (dt <= 0.0 || dt > 0.1) dt = 0.05; 
                lastTimePID = now;

                // 1. IR (Lateral) PID Math
                integralIR += errorIR * dt;
                float derivativeIR = (errorIR - prevErrorIR) / dt;
                prevErrorIR = errorIR;
                
                float pidIR = (errorIR * kP_IR) + (integralIR * kI_IR) + (derivativeIR * kD_IR);

                // 2. IMU (Heading) PID Math
                integralIMU += errorIMU * dt;
                float derivativeIMU = (errorIMU - prevErrorIMU) / dt;
                prevErrorIMU = errorIMU;

                
                float pidIMU = (errorIMU * kP_IMU) + (integralIMU * kI_IMU) + (derivativeIMU * kD_IMU);

                // 3. Fusion / Motor Mixing
                int centerBaseSpeed = BASE_SPEED * 5 / 4;
                
                // Add IR to push to center, Subtract IMU to resist twisting
                int leftSpeed = constrain(centerBaseSpeed + pidIR - pidIMU, 50, 255);
                int rightSpeed = constrain(centerBaseSpeed - pidIR + pidIMU, 50, 255);
                
                motors.setM1Speed(leftSpeed);
                motors.setM2Speed(rightSpeed);
            }
            break;

        case APPROACH_TAG:
            {
                Serial.println("Approach TagG      ");
                int leftSpeed = constrain(BASE_SPEED + (targetTagError * kP_pixy), 50, 255);
                int rightSpeed = constrain(BASE_SPEED - (targetTagError * kP_pixy), 50, 255);
                motors.setM1Speed(leftSpeed);
                motors.setM2Speed(rightSpeed);
                
                // Keep syncing heading dynamically so when the tag vanishes, it locks in the angle
                targetHeading = pos.yaw; 
            }
            break;
    }
}