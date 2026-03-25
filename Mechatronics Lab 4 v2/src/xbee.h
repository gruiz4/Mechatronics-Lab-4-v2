#ifndef ZIGBEE_TRACKING_H
#define ZIGBEE_TRACKING_H

#include <Arduino.h>
// ===== CONFIGURATION =====
// Comment out the next line to only print X and Y
#define DEBUG

// In DEBUG mode, comment out the next line to hide invalid messages
// (they are still counted in stats, just not printed)
#define SHOW_INVALID

// --- FORMAT FLAG ---
// Define BROADCAST_FORMAT to parse the coordinator's broadcast payload:
//   >MTTTTRXXXYYY...CC;
// Comment it out to use the old comma-separated format:
//   matchByte,gameTime,X,Y
#define BROADCAST_FORMAT

// Set your robot ID here (the letter assigned to this bot's XBee module)
// In broadcast mode this must match the letter the coordinator assigns
// (chr(61 + tag_id) in zigbee.py, so tag_id 4 → 'A', 5 → 'B', etc.)
#define ROBOT_ID  'K'

// --- FILTER FLAG (broadcast mode only) ---
// Define FILTER_MY_ROBOT to only process messages containing ROBOT_ID.
// Comment it out to accept all valid broadcasts and display every robot.
#define FILTER_MY_ROBOT

// Timeout (ms) – if no new byte arrives within this window, treat
// whatever is in the buffer as a complete message.
#define RX_TIMEOUT_MS  5

// ===== END CONFIGURATION =====


#ifdef BROADCAST_FORMAT
#define MAX_ROBOTS 15

// Store all robots from the broadcast message
struct RobotEntry {
  char letter;
  int  x;
  int  y;
};

extern RobotEntry robots[MAX_ROBOTS];
extern int numRobots;
#endif

// Parsed fields
extern int   matchByte;
extern long  gameTime;
extern int   xPos;
extern int   yPos;

// Previous position & time for speed calculation
extern int   prevX;
extern int   prevY;
extern unsigned long prevTimeMicros;
extern float speed;

// Stats (used in DEBUG mode)
extern unsigned long totalResponses;
extern unsigned long validCoords;
extern unsigned long invalidResponses;

#ifndef BROADCAST_FORMAT
// Only used in legacy query mode
extern unsigned long totalQueries;
#endif

// Sampling rate measurement
extern unsigned long hzCounter;
extern unsigned long lastHzTime;
extern float         samplingRateHz;

// Buffer for incoming XBee data
extern char rxBuffer[128];
extern int  rxIndex;

// Timestamp of last received byte
extern unsigned long lastRxTime;

// ---------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------
int extractDigits(const char* buf, int len, int &pos, int numDigits);

#ifdef BROADCAST_FORMAT
bool parseBroadcast(const char* buf);
#endif

bool parseCsv(const char* buf);
bool parseResponse(const char* buf);
void processMessage();

// Core flow (rename these if calling from a main sketch's own setup/loop)
void setup();
void loop();

#endif // ZIGBEE_TRACKING_H