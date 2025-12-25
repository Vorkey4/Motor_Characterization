#include "esp_timer.h"

// definitions
const int MOTOR_PIN = 23;
const int ADC_PIN   = 34;

const int RUNS = 126;

const uint32_t motor_on_ms  = 2000;
const uint32_t motor_off_ms = 2000;

// delay table due to the need for precise time samples
const uint32_t sample_delay_us[RUNS] = {
   0,  2,  4,  6,  8, 10, 12, 14, 16, 18,
  20, 22, 24, 26, 28, 30, 32, 34, 36, 38,
  40, 42, 44, 46, 48, 50, 52, 54, 56, 58,
  60, 62, 64, 66, 68, 70, 72, 74, 76, 78,
  80, 82, 84, 86, 88, 90, 92, 94, 96, 98,
 100,102,104,106,108,110,112,114,116,118,
 120,122,124,126,128,130,132,134,136,138,
 140,142,144,146,148,150,152,154,156,158,
 160,162,164,166,168,170,172,174,176,178,
 180,182,184,186,188,190,192,194,196,198,
 200,202,204,206,208,210,212,214,216,218,
 220,222,224,226,228,230,232,234,236,238,
 240,242,244,246,248,250
};

// table with results
uint16_t adcResults[RUNS];

// loop state
enum State { START_RUN, MOTOR_ON, MOTOR_OFF, DONE };
State state = START_RUN;

int runIndex = 0;

uint64_t motorStartUs = 0;
uint64_t sampleTimeUs = 0;
bool sampleTaken = false;

unsigned long stateStartMs = 0;

// setup
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  pinMode(ADC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  Serial.println("Final test: 2s ON / 2s OFF, sample once at 0–250us (2us steps)");
}

// print data
void printData() {
  Serial.println("\n--- DATA START (Delay_us, ADC_Value) ---");
  for (int i = 0; i < RUNS; i++) {
    Serial.print(sample_delay_us[i]);
    Serial.print(",");
    Serial.println(adcResults[i]);
  }
  Serial.println("--- DATA END ---");
}

// loop block
void loop() {
  switch (state) {

    case START_RUN:
      if (runIndex >= RUNS) {
        state = DONE;
        break;
      }

      digitalWrite(MOTOR_PIN, HIGH);

      motorStartUs = esp_timer_get_time();
      sampleTimeUs = motorStartUs + sample_delay_us[runIndex];
      sampleTaken  = false;

      stateStartMs = millis();
      state = MOTOR_ON;
      break;

    case MOTOR_ON: {
      uint64_t nowUs = esp_timer_get_time();

      // Exactly ONE ADC sample at scheduled offset
      if (!sampleTaken && nowUs >= sampleTimeUs) {
        adcResults[runIndex] = analogRead(ADC_PIN);
        sampleTaken = true;
      }

      // Motor ON duration
      if (millis() - stateStartMs >= motor_on_ms) {
        digitalWrite(MOTOR_PIN, LOW);

        // Safety fallback
        if (!sampleTaken) {
          adcResults[runIndex] = analogRead(ADC_PIN);
        }

        stateStartMs = millis();
        state = MOTOR_OFF;
      }
      break;
    }

    case MOTOR_OFF:
      if (millis() - stateStartMs >= motor_off_ms) {
        runIndex++;
        state = START_RUN;
      }
      break;

    case DONE:
      printData();
      while (true) { }  // stop forever
  }
}
