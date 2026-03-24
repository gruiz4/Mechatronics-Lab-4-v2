// Code used in the navigation and planning of maze robot
#include "drivers.h"

#define map_size 500
struct position{
    
int global_map[map_size][map_size]; /* Map is in cm. 1 when there is an obstacle measured 
there, 0 when open  */
int temp_map[map_size][map_size];
int roll;
int pitch;
int yaw;
float accel[3];
int D_L;
int D_R;
int D;
};



enum State { 
  SEARCHING,        // No walls, no tags
  DRIVE_STRAIGHT,   // Driving normally in a lane
  CENTERING,        // Adjusting to stay in middle of lane
  PIVOT_LEFT_DIST,  // Obstacle in front, turning left
  PIVOT_RIGHT_DIST, // Obstacle in front, turning right
  TURN_TAG,         // Executing a color-based turn
  REVERSING,        // Backing up because we are stuck
  APPROACH_TAG      // Driving towards a distant tag
};
