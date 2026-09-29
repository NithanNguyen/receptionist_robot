#include <Arduino.h>

// ============================================================================
//  ESP32 Ultrasonic Bridge - 6-channel HC-SR04 sequential scanner
//  Fixes acoustic cross-talk / early-echo artefacts observed in reflective
//  corridor environments (all sensors latching a stable ~0.8 m false reading).
//
//  Key changes vs. previous firmware:
//    1. TRUE sequential scanning: only ONE sensor is ever mid-measurement.
//       currentSensor advances only after that sensor completes or times out.
//    2. SETTLING_MS silence window after every measurement, so the 40 kHz burst
//       (and its corridor reverberation) fully decays before the next sensor
//       fires. This is the root-cause fix for cross-talk.
//    3. Range clamping: echoes above MAX_VALID_CM -> NO_ECHO_CM (clear);
//       echoes below the HC-SR04 physical floor (~2 cm) -> NO_ECHO_CM.
//       Real near obstacles (e.g. 15 cm) are preserved -- no near blind zone.
//    4. Frame period is now decoupled from the scan: the frame is emitted at a
//       fixed rate carrying the most recent completed value per sensor.
// ============================================================================

#define JETSON_BAUD 115200
#define JETSON_TX   1
#define JETSON_RX   3

// Ultrasonics's pin
// Echo:           |  Trig
// top-right: G27  |  G4
// top-left:  G33  |  G19
// mid-right: G14  |  G0
// mid-left:  G23  |  G22
// bot-right: G12  |  G2
// bot-left:  G26  |  G5

// ---- Pin map ----------------
const uint8_t TRIG[] = {4, 19, 0, 22, 2, 5};
const uint8_t ECHO[] = {27, 33, 14, 23, 12, 26};

#define NUM_SENSORS 6

// ---- Tunable parameters ----------------------------------------------------
#define TIME_SEND_MS     100     // frame emit period [ms] (10 Hz output cap)
#define SETTLING_US      30000UL // silence after each measurement [us] (~10 m decay)
#define ECHO_START_TO_US 25000UL // max wait for echo rising edge [us] (~4.3 m)
#define ECHO_END_TO_US   25000UL // max echo pulse width [us] (~4.3 m one-way)
#define MIN_VALID_CM     1       // HC-SR04 physical floor only; real near
                                 // obstacles (e.g. 15 cm) are KEPT, not discarded
#define MAX_VALID_CM     200     // HC-SR04 practical ceiling; above -> NO_ECHO_CM
#define NO_ECHO_CM       999     // sentinel: clear / no return

// ---- Per-sensor state machine ----------------------------------------------
enum SonicState {
  TRIG_LOW_S,       // pull trig low to settle before pulse
  TRIG_HIGH_S,      // 10 us high pulse
  WAIT_ECHO_START,  // wait for echo rising edge
  WAIT_ECHO_END,    // measure echo high duration
  SETTLING          // enforced silence before next sensor
};

struct SonicSensor {
  SonicState    state    = TRIG_LOW_S;
  unsigned long stateT   = 0;   // timestamp marking current state entry [us]
  unsigned long echoT    = 0;   // echo rising-edge timestamp [us]
  long          distCM   = NO_ECHO_CM;  // last completed measurement
};

SonicSensor   sensors[NUM_SENSORS];
uint8_t       currentSensor = 0;
unsigned long iTimeSend      = 0;

// Advance the round-robin to the next sensor and prime it.
static inline void advanceSensor() {
  currentSensor = (currentSensor + 1) % NUM_SENSORS;
  sensors[currentSensor].state  = TRIG_LOW_S;
  sensors[currentSensor].stateT = micros();
}

// Non-blocking measurement step for the currently active sensor.
void serviceActiveSensor() {
  SonicSensor& s = sensors[currentSensor];
  const uint8_t i = currentSensor;
  unsigned long now = micros();

  switch (s.state) {

    case TRIG_LOW_S:
      digitalWrite(TRIG[i], LOW);
      if (now - s.stateT >= 3) {              // >=2 us settle, use 3 for margin
        digitalWrite(TRIG[i], HIGH);
        s.stateT = now;
        s.state  = TRIG_HIGH_S;
      }
      break;

    case TRIG_HIGH_S:
      if (now - s.stateT >= 10) {             // 10 us trigger pulse
        digitalWrite(TRIG[i], LOW);
        s.stateT = now;
        s.state  = WAIT_ECHO_START;
      }
      break;

    case WAIT_ECHO_START:
      if (digitalRead(ECHO[i]) == HIGH) {
        s.echoT = now;
        s.state = WAIT_ECHO_END;
      } else if (now - s.stateT > ECHO_START_TO_US) {
        s.distCM = NO_ECHO_CM;                // no echo -> treat as clear
        s.stateT = now;
        s.state  = SETTLING;
      }
      break;

    case WAIT_ECHO_END:
      if (digitalRead(ECHO[i]) == LOW) {
        long d = (long)(now - s.echoT) / 58;  // us -> cm
        if (d < MIN_VALID_CM) {
          // Below HC-SR04 physical floor (~2 cm): unmeasurable, treat as clear.
          // NOTE: this does NOT create a near blind zone -- a real 15 cm
          // obstacle yields d=15 (>= MIN_VALID_CM) and is reported correctly.
          s.distCM = NO_ECHO_CM;
        } else if (d > MAX_VALID_CM) {
          s.distCM = NO_ECHO_CM;         // beyond range -> clear (-> inf on Jetson)
        } else {
          s.distCM = d;
        }
        s.stateT = now;
        s.state  = SETTLING;
      } else if (now - s.echoT > ECHO_END_TO_US) {
        s.distCM = NO_ECHO_CM;                // echo stuck high -> discard
        s.stateT = now;
        s.state  = SETTLING;
      }
      break;

    case SETTLING:
      // Enforced acoustic silence: let the burst + corridor reverb decay
      // before the next sensor fires. This is the core cross-talk fix.
      if (now - s.stateT >= SETTLING_US) {
        advanceSensor();
      }
      break;
  }
}

void setup() {
  Serial.begin(JETSON_BAUD, SERIAL_8N1, JETSON_RX, JETSON_TX);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    digitalWrite(TRIG[i], LOW);
  }
  sensors[currentSensor].stateT = micros();
}

void loop() {
  // Only the active sensor is serviced -> guarantees one burst at a time.
  serviceActiveSensor();

  unsigned long nowMs = millis();
  if (nowMs - iTimeSend < TIME_SEND_MS) return;
  iTimeSend = nowMs;

  long vals[NUM_SENSORS];
  for (uint8_t i = 0; i < NUM_SENSORS; i++) vals[i] = sensors[i].distCM;

  uint8_t chk = (uint8_t)vals[0];
  for (uint8_t i = 1; i < NUM_SENSORS; i++) chk ^= (uint8_t)vals[i];

  Serial.print('$');
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(vals[i]);
    Serial.print(i < NUM_SENSORS - 1 ? ',' : '*');
  }
  Serial.println(chk);
}