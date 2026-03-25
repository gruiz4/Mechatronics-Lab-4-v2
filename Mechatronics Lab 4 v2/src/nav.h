#ifndef NAV_H
#define NAV_H

#include "drivers.h"
#include "xbee.h"

#define map_size 1

#define XbeeXtol 5
#define XbeeYtol 5


enum State { 
  SEARCHING,        // No walls, no tags
  DRIVE_STRAIGHT,   // Driving normally in a lane
  CENTERING,        // Adjusting to stay in middle of lane
  PIVOT_LEFT_DIST,  // Obstacle in front, turning left
  PIVOT_RIGHT_DIST, // Obstacle in front, turning right
  TURN_TAG,         // Executing a color-based turn
  REVERSING,        // Backing up because we are stuck
  APPROACH_TAG,      // Driving towards a distant tag
  PAUSE             // State used when it finishes the course
};

struct position {
    int global_map[map_size][map_size]; /* Map is in cm. 1 when there is an obstacle measured there, 0 when open */
    int temp_map[map_size][map_size];
    float roll;
    float pitch;
    float yaw;
    float accel[3];
    float D_L;
    float D_R;
    float D;
    State currentState;
    int xbeeX;
    int xbeeY;
    int gameByte;
};

// Expose current state for shared usage across files
// extern State currentState;


// Global position tracking object
extern struct position pos;
// Navigation state variables
extern unsigned long tagTimeout;
extern unsigned long tagInterval;
extern unsigned long ignoreCorrectionsUntil;
extern float targetTagTurn;
extern float targetTagError;

extern float targetHeading;

// Function Declarations
void getHeadingerror();
// void updatePosition(float r, float p, float y, float ax, float ay, float az); //removed. see commit.
void navigate();
bool check_end(int XFinish, int YFinish, int Xnow, int Ynow);
#endif