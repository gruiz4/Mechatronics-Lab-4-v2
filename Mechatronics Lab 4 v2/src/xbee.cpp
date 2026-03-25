#include "xbee.h" 

// ---------------------------------------------------------------
// Global Variable Definitions (initialized here)
// ---------------------------------------------------------------

// Parsed fields
int   matchByte  = 0;
long  gameTime   = 0;
int   xPos       = 0;
int   yPos       = 0;

#ifdef BROADCAST_FORMAT
// Store all robots from the broadcast message
RobotEntry robots[MAX_ROBOTS];
int numRobots = 0;
#endif

// Previous position & time for speed calculation
int   prevX      = 0;
int   prevY      = 0;
unsigned long prevTimeMicros = 0;
float speed      = 0.0;  // units per second

// Stats (used in DEBUG mode)
unsigned long totalResponses   = 0;
unsigned long validCoords      = 0;
unsigned long invalidResponses = 0;

#ifndef BROADCAST_FORMAT
// Only used in legacy query mode
unsigned long totalQueries     = 0;
#endif

// Sampling rate measurement
unsigned long hzCounter       = 0;
unsigned long lastHzTime       = 0;
float         samplingRateHz   = 0.0;

// Buffer for incoming XBee data
char rxBuffer[128];
int  rxIndex = 0;

// Timestamp of last received byte (for timeout detection)
unsigned long lastRxTime = 0;

// ---------------------------------------------------------------
// Helper: extract N decimal digits from buf starting at pos
// Returns the integer value, advances pos by N.
// Returns -1 if any character is not a digit.
// ---------------------------------------------------------------
int extractDigits(const char* buf, int len, int &pos, int numDigits) {
  int value = 0;
  for (int i = 0; i < numDigits; i++) {
    if (pos >= len) return -1;
    char c = buf[pos++];
    if (c < '0' || c > '9') return -1;
    value = value * 10 + (c - '0');
  }
  return value;
}

#ifdef BROADCAST_FORMAT
// ---------------------------------------------------------------
// Parse broadcast format:  >MTTTTRXXXYYY...CC;
// ---------------------------------------------------------------
bool parseBroadcast(const char* buf) {
  int len = strlen(buf);

  // Minimum valid message: >M TTTT R XXX YYY CC ;  = 13 chars
  if (len < 13) return false;

  // Check start and end bytes
  if (buf[0] != '>')       return false;
  if (buf[len - 1] != ';') return false;

  // --- Verify checksum ---
  int txChk = (buf[len - 3] - '0') * 10 + (buf[len - 2] - '0');
  if (buf[len - 3] < '0' || buf[len - 3] > '9') return false;
  if (buf[len - 2] < '0' || buf[len - 2] > '9') return false;

  int calcChk = 0;
  for (int i = 0; i < len - 3; i++) { 
    calcChk += (unsigned char)buf[i];
  }
  calcChk += ';'; 
  calcChk %= 64;

  if (calcChk != txChk) return false;

  // --- Parse header ---
  int pos = 1; 

  int mBit = extractDigits(buf, len, pos, 1);
  if (mBit < 0) return false;

  int mTime = extractDigits(buf, len, pos, 4);
  if (mTime < 0) return false;

  matchByte = mBit;
  gameTime  = mTime;

  // --- Scan robot entries ---
  int dataEnd = len - 3; 
  bool foundSelf = false;
  numRobots = 0;

  while (pos + 7 <= dataEnd && numRobots < MAX_ROBOTS) {
    char robotLetter = buf[pos++];

    int rx = extractDigits(buf, len, pos, 3);
    if (rx < 0) return false;

    int ry = extractDigits(buf, len, pos, 3);
    if (ry < 0) return false;

    robots[numRobots].letter = robotLetter;
    robots[numRobots].x      = rx;
    robots[numRobots].y      = ry;
    numRobots++;

    if (robotLetter == ROBOT_ID) {
      xPos      = rx;
      yPos      = ry;
      foundSelf = true;
    }
  }

#ifdef FILTER_MY_ROBOT
  return foundSelf; 
#else
  return (numRobots > 0);
#endif
}
#endif

// ---------------------------------------------------------------
// Parse old comma-separated response
// ---------------------------------------------------------------
bool parseCsv(const char* buf) {
  int   f1 = 0;
  long  f2 = 0;
  int   f3 = 0;
  int   f4 = 0;

  int matched = sscanf(buf, "%d,%ld,%d,%d", &f1, &f2, &f3, &f4);
  if (matched == 4) {
    matchByte = f1;
    gameTime  = f2;
    xPos      = f3;
    yPos      = f4;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------
// Top-level parser
// ---------------------------------------------------------------
bool parseResponse(const char* buf) {
#ifdef BROADCAST_FORMAT
  return parseBroadcast(buf);
#else
  return parseCsv(buf);
#endif
}

// ... [Keep everything above processMessage() exactly the same] ...

// ---------------------------------------------------------------
// Process a completed message
// ---------------------------------------------------------------
bool processMessage() {
  if (rxIndex == 0) return false;

  rxBuffer[rxIndex] = '\0'; 
  totalResponses++;

  bool success = parseResponse(rxBuffer);

  if (success) {
    validCoords++;
    hzCounter++;

    // ---------- OUTPUT ----------
#ifdef DEBUG
    float successPct = (totalResponses > 0)
                         ? (validCoords * 100.0 / totalResponses)
                         : 0.0;

#ifdef BROADCAST_FORMAT
    for (int i = 0; i < numRobots; i++) {
      bool isSelf = (robots[i].letter == ROBOT_ID);

#ifdef FILTER_MY_ROBOT
      if (!isSelf) continue; 
#endif

      if (isSelf) {
        unsigned long nowMicros = micros();
        float dtSec = (nowMicros - prevTimeMicros) / 1000000.0;
        if (dtSec > 0.001 && prevTimeMicros > 0) {
          float dx = xPos - prevX;
          float dy = yPos - prevY;
          speed = sqrt(dx * dx + dy * dy) / dtSec;
        }
        prevX = xPos;
        prevY = yPos;
        prevTimeMicros = nowMicros;
      }

      Serial.print(isSelf ? '*' : ' ');
      Serial.print(robots[i].letter);  Serial.print(" | ");
      Serial.print(matchByte);         Serial.print(" | ");
      Serial.print(robots[i].x);       Serial.print(" | ");
      Serial.print(robots[i].y);       Serial.print(" | ");
      if (isSelf) {
        Serial.print(speed, 1);
      } else {
        Serial.print('-');
      }
      Serial.print(" | ");
      Serial.print(samplingRateHz, 1); Serial.print(F(" Hz | "));
      Serial.print(gameTime);          Serial.print(" | ");
      Serial.print(totalResponses);    Serial.print(" | ");
      Serial.print(validCoords);       Serial.print(" | ");
      Serial.print(invalidResponses);  Serial.print(" | ");
      Serial.print(successPct, 1);     Serial.println('%');
    }
#else
    unsigned long nowMicros = micros();
    float dtSec = (nowMicros - prevTimeMicros) / 1000000.0;
    if (dtSec > 0.001 && prevTimeMicros > 0) {
      float dx = xPos - prevX;
      float dy = yPos - prevY;
      speed = sqrt(dx * dx + dy * dy) / dtSec;
    }
    prevX = xPos;
    prevY = yPos;
    prevTimeMicros = nowMicros;

    Serial.print((char)ROBOT_ID);    Serial.print(" | ");
    Serial.print(matchByte);         Serial.print(" | ");
    Serial.print(xPos);              Serial.print(" | ");
    Serial.print(yPos);              Serial.print(" | ");
    Serial.print(speed, 1);          Serial.print(" | ");
    Serial.print(samplingRateHz, 1); Serial.print(F(" Hz | "));
    Serial.print(gameTime);          Serial.print(" | ");
    Serial.print(totalResponses);    Serial.print(" | ");
    Serial.print(validCoords);       Serial.print(" | ");
    Serial.print(invalidResponses);  Serial.print(" | ");
    Serial.print(successPct, 1);     Serial.println('%');
#endif
#else
    Serial.print(xPos);
    Serial.print(" , ");
    Serial.println(yPos);
#endif

  } else {
    invalidResponses++;

#if defined(DEBUG) && defined(SHOW_INVALID)
    Serial.print(F("[INVALID] raw: \""));
    Serial.print(rxBuffer);
    Serial.println(F("\""));
#endif
  }

  rxIndex = 0; 
  return success;
}

// ---------------------------------------------------------------
// Renamed from setup() to setupXBee()
// ---------------------------------------------------------------
void setupXBee() {
  // Serial.begin and Serial2.begin are already in main.cpp, 
  // so you can leave them here or remove them. 
  delay(500);

  Serial.println(F("=== XBee Tracking Started ==="));
  Serial.print(F("Robot ID: "));
  Serial.println((char)ROBOT_ID);

#ifdef BROADCAST_FORMAT
  Serial.println(F("FORMAT: BROADCAST (>MTTTTRXXXYYY...CC;)"));
#else
  Serial.println(F("FORMAT: CSV (matchByte,gameTime,X,Y)"));
#endif

#ifdef DEBUG
  Serial.println(F("MODE: DEBUG (full stats)"));
  Serial.println(F("RobotID | MatchByte | X | Y | Speed | Hz | GameTime | Responses | ValidCoords | Invalid | CoordSuccess%"));
#else
  Serial.println(F("MODE: MINIMAL (X, Y only)"));
  Serial.println(F("X , Y"));
#endif

  Serial.println(F("----------------------------"));

  prevTimeMicros = micros();
  lastRxTime     = millis();
  lastHzTime     = millis();
}

// ---------------------------------------------------------------
// Standalone parsing function
// ---------------------------------------------------------------
bool fetchXBeePosition(int &outX, int &outY) {
  bool newPositionReady = false;

#ifndef BROADCAST_FORMAT
  Serial2.print('?');
  totalQueries++;
#endif

  unsigned long now = millis();
  if (now - lastHzTime >= 1000) {
    samplingRateHz = (float)hzCounter;
    hzCounter = 0;
    lastHzTime = now;
  }

  while (Serial2.available()) {
    char c = Serial2.read();
    lastRxTime = millis();

#ifdef BROADCAST_FORMAT
    if (c == ';') {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) {
        rxBuffer[rxIndex++] = c;
      }
      if (processMessage()) newPositionReady = true;
    } else if (c == '>') {
      rxIndex = 0;
      rxBuffer[rxIndex++] = c;
    } else {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) {
        rxBuffer[rxIndex++] = c;
      } else {
        rxIndex = 0; 
      }
    }
#else
    if (c == '\n' || c == '\r') {
      if (processMessage()) newPositionReady = true;
    } else {
      if (rxIndex < (int)(sizeof(rxBuffer) - 1)) {
        rxBuffer[rxIndex++] = c;
      } else {
        if (processMessage()) newPositionReady = true;
      }
    }
#endif
  }

  if (rxIndex > 0 && (millis() - lastRxTime >= RX_TIMEOUT_MS)) {
    if (processMessage()) newPositionReady = true;
  }

  if (newPositionReady) {
    outX = xPos;
    outY = yPos;
  }

  return newPositionReady;
}