// Sketch for the peakock project.
//
// It reads two sensors, a moisture sensor on analog pin 0 and a PSR
// on analog pin 1. The moisture sensor reads the vaginal moisture and
// the PSR reads intentional contractions. Two servo motors control
// the peacock tail: when the moisture levels grow it opens and flash
// by intentional contractions.

#include <Servo.h>

#define MOISTURE_PIN 0
#define PSR_PIN 1
#define SERVO_A_PIN 9
#define SERVO_B_PIN 10

#define MOISTURE_MIN 0
#define MOISTURE_MAX 960
#define PSR_THRESHOLD 600

#define CONTRACTION_REPETITIONS 3  // Number of flashs per contraction
#define CONTRACTION_UPDATE_TIME 4  // Time (ms) between value updates
#define BUFFER_N 100               // The number of readings to average
#define MAX_DEGREES 80

Servo servoA;
Servo servoB;

int pos[BUFFER_N];  // Averaging accross lots of readings for smoother value
int pos_insertion_index = 0;
int contractionValue = 0;
int contractionDelta = 1;
int contractionIndex = 0;
long contractionLastUpdate = 0;

void setup() {
  servoA.attach(9);
  servoB.attach(10);

  // Init positions buffer
  for (int i = 0; i < BUFFER_N; i++) {
    pos[i] = 0;
  }
}

void loop() {
  // Read sensors
  int moistureValue = analogRead(MOISTURE_PIN);
  int psrValue = analogRead(PSR_PIN);
  long now = millis();

  // Map and constrain values
  moistureValue = map(moistureValue, MOISTURE_MIN, MOISTURE_MAX, 0, MAX_DEGREES);
  moistureValue = constrain(moistureValue, 0, MAX_DEGREES);

  // Update position buffer
  pos[pos_insertion_index] = moistureValue;
  pos_insertion_index = (pos_insertion_index + 1) % BUFFER_N;

  // Get mean
  int meanPos = 0;
  for (int i = 0; i < BUFFER_N; i++) {
    meanPos += pos[i];
  }
  meanPos = int(meanPos / BUFFER_N);

  // Calculate contraction value
  if (psrValue > PSR_THRESHOLD && contractionIndex == 0) {
    contractionIndex = 1;
  }
  if ((contractionIndex != 0) && \
      (now - contractionLastUpdate > CONTRACTION_UPDATE_TIME)) {
    contractionLastUpdate = now;
    contractionValue += contractionDelta;
    if (meanPos + contractionValue >= MAX_DEGREES) {
      contractionValue = MAX_DEGREES - meanPos;
      contractionDelta *= -1;
    } else if (contractionValue <= 0) {
      contractionValue = 0;
      contractionDelta *= -1;
      contractionIndex = (contractionIndex + 1) % (CONTRACTION_REPETITIONS + 1);
    }
  }

  // Move servos
  int currentPos = meanPos + contractionValue;
  servoA.write(currentPos);
  servoB.write(90 - currentPos);
}
