// install https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library library
// first

uint8_t correct_heartRate; // Heartrate value
uint8_t correct_spo2;      // SPO2 value

// ECG sensor part

#define ECGA1 10
#define ECGA2 11
#define ECGLED 8
#define ECGBTN 4
void displayError();

class AdaptiveECGFilter {
private:
  // Filter parameters
  float baselineLPF;    // Low-pass factor for baseline estimation
  float spikeThreshold; // Threshold to detect spikes
  float recoveryRate;   // Rate at which filter recovers after spikes

  // Filter state variables
  float baseline;         // Current baseline estimate
  float lastValue;        // Previous input value
  float lastOutput;       // Previous output value
  bool spikeDetected;     // Spike detection flag
  int spikeRecoveryCount; // Recovery counter after spike

public:
  AdaptiveECGFilter(float baselineLPF = 0.05, float spikeThreshold = 50.0,
                    float recoveryRate = 0.8) {
    this->baselineLPF = baselineLPF;
    this->spikeThreshold = spikeThreshold;
    this->recoveryRate = recoveryRate;

    // Initialize state
    baseline = 0.0;
    lastValue = 0.0;
    lastOutput = 0.0;
    spikeDetected = false;
    spikeRecoveryCount = 0;
  }

  float processValue(float input) {
    // Calculate derivative to detect rapid changes
    float derivative = input - lastValue;

    // Spike detection
    if (abs(derivative) > spikeThreshold) {
      spikeDetected = true;
      spikeRecoveryCount = 5; // Number of samples for recovery phase
    }

    // Baseline adaptation - slower during spikes, faster during normal segments
    float adaptRate = spikeDetected ? baselineLPF * 0.1 : baselineLPF;
    baseline = baseline * (1.0 - adaptRate) + input * adaptRate;

    // Gradually reduce spike detection flag
    if (spikeRecoveryCount > 0) {
      spikeRecoveryCount--;
    } else {
      spikeDetected = false;
    }

    // Remove baseline from signal
    float centered = input - baseline;

    // Smooth transitions after spikes
    float output;
    if (spikeDetected) {
      // During spikes, use limited filtering
      output = centered;
    } else {
      // Normal filtering - blend with previous output for smoothness
      output = centered * (1 - recoveryRate) + lastOutput * recoveryRate;
    }

    // Update state
    lastValue = input;
    lastOutput = output;

    return output;
  }
};
AdaptiveECGFilter *highPassFilter;

void ecgSetup() {

  if (!(highPassFilter = new AdaptiveECGFilter(0.1))) {
    Serial.println(F("highPassFilter memory not allocated!"));
    displayError();
  }

  //    Serial.flush();
  pinMode(ECGA1, INPUT);
  pinMode(ECGA2, INPUT);
  while (digitalRead(ECGBTN))
    ;
}

#define ECGOFFSET 50

void ecgDataCollect() {
  digitalWrite(ECGLED, HIGH);
  Serial.print(correct_heartRate);
  Serial.print(F(","));
  Serial.println(correct_spo2);
  for (uint16_t i = 0; i < 500 + ECGOFFSET; i++) {
    if ((digitalRead(ECGA1) == 1) || (digitalRead(ECGA2) == 1))
      i--;
    else {
      int rawValue = analogRead(A0);
      float filteredValue = highPassFilter->processValue(rawValue);
      if (i > (0 + ECGOFFSET)) {
        //          Serial.print(rawValue);
        //          Serial.print(",");
        Serial.println(filteredValue / 10);
      }
    }
    delay(20);
  }

  digitalWrite(ECGLED, LOW);
}

void ecgCleanup() { free(highPassFilter); }

// MAX30102 sensor part

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Wire.h>

#define RATE_SIZE 4 // Increase this for more averaging. 4 is good.
                    // Array of heart rates

MAX30105 *particleSensor;

#define MAX_BRIGHTNESS 255

#define LEDBRIGHTNESS 60 // Options: 0=Off to 255=50mA
#define SAMPLEAVERAGE 16 // Options: 1, 2, 4, 8, 16, 32
#define LEDMODE 2 // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
#define SAMPLERATE 800 // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
#define PULSEWIDTH 411 // Options: 69, 118, 215, 411
#define ADCRANGE 4096  // Options: 2048, 4096, 8192, 16384

#define PULSELED 11 // Must be on PWM pin
#define READLED 13  // Blinks with each data read
#define MAXLED 2
#define MAXBTN 7

#define BUFFERLENGTH                                                           \
  60 // data length //buffer length of 100 stores 4 seconds of samples running
     // at 25sps

void spo2calc() {
  //    Serial.begin(115200); // initialize serial communication at 115200 bits
  //    per second:
  uint16_t *irBuffer;  // infrared LED sensor data
  uint16_t *redBuffer; // red LED sensor data
  if (!(particleSensor = (MAX30105 *)malloc(sizeof(MAX30105)))) {
    Serial.println(F("particleSensor memory not allocated!"));
    displayError();
  }
  if (!(irBuffer = (uint16_t *)malloc(sizeof(uint16_t) * BUFFERLENGTH))) {
    Serial.println(F("irBuffer memory not allocated!"));
    displayError();
  }
  if (!(redBuffer = (uint16_t *)malloc(sizeof(uint16_t) * BUFFERLENGTH))) {
    Serial.println(F("redBuffer memory not allocated!"));
    displayError();
  }
  pinMode(PULSELED, OUTPUT);
  pinMode(READLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor->begin(
          Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    displayError();
    while (1)
      ;
  }

  //    Serial.println(F("Attach sensor to finger with rubber band. Press any
  //    key to start conversion"));
  while (digitalRead(MAXBTN) == HIGH)
    ; // wait until user presses a key
  //  Serial.read();

  particleSensor->setup(LEDBRIGHTNESS, SAMPLEAVERAGE, LEDMODE, SAMPLERATE,
                        PULSEWIDTH,
                        ADCRANGE); // Configure sensor with these settings
  int32_t spo2;                    // SPO2 value
  int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
  int32_t heartRate;     // heart rate value
  int8_t validHeartRate; // indicator to show if the heart rate calculation is
                         // valid
  digitalWrite(MAXLED, HIGH);
  // read the first 100 samples, and determine the signal range
  for (byte i = 0; i < 5; i++) {
    for (byte i = 0; i < BUFFERLENGTH; i++) {
      while (particleSensor->available() == false) // do we have new data?
        particleSensor->check(); // Check the sensor for new data

      redBuffer[i] = particleSensor->getRed();
      irBuffer[i] = particleSensor->getIR();
      particleSensor->nextSample(); // We're finished with this sample so move
                                    // to next sample
    }

    // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of
    // samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFERLENGTH, redBuffer,
                                           &spo2, &validSPO2, &heartRate,
                                           &validHeartRate);

    // Continuously taking samples from MAX30102.  Heart rate and SpO2 are
    // calculated every 1 second

    // dumping the first 25 sets of samples in the memory and shift the last 75
    // sets of samples to the top
    for (byte i = BUFFERLENGTH / 4; i < BUFFERLENGTH; i++) {
      redBuffer[i - BUFFERLENGTH / 4] = redBuffer[i];
      irBuffer[i - BUFFERLENGTH / 4] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (byte i = BUFFERLENGTH * 3 / 4; i < BUFFERLENGTH; i++) {
      while (particleSensor->available() == false) // do we have new data?
        particleSensor->check(); // Check the sensor for new data

      digitalWrite(
          READLED,
          !digitalRead(READLED)); // Blink onboard LED with every data read

      redBuffer[i] = particleSensor->getRed();
      irBuffer[i] = particleSensor->getIR();
      particleSensor->nextSample(); // We're finished with this sample so move
                                    // to next sample
    }
    //  Serial.print(F(" HR="));
    //  Serial.print(heartRate/2, DEC);
    ////        Serial.print(F(", HRvalid="));
    ////        Serial.print(validHeartRate, DEC);
    //  Serial.print(F("; SPO2="));
    //  Serial.println(spo2, DEC);
    if (validSPO2)
      correct_spo2 = spo2;
  }
  free(redBuffer);
  free(irBuffer);
  free(particleSensor);
}

class HeartRate {
private:
  int16_t cbuf[32];
  uint8_t offset = 0;
  int16_t IR_AC_Max = 20;
  int16_t IR_AC_Min = -20;

  int16_t IR_AC_Signal_Current = 0;
  int16_t IR_AC_Signal_Previous;
  int16_t IR_AC_Signal_min = 0;
  int16_t IR_AC_Signal_max = 0;
  int16_t IR_Average_Estimated;

  int16_t positiveEdge = 0;
  int16_t negativeEdge = 0;
  int32_t ir_avg_reg = 0;
  bool beatDetected = false;
  static const uint16_t FIRCoeffs[12] = {172,  321,  579,  927,  1360, 1858,
                                         2390, 2916, 3391, 3768, 4012, 4096};
  int16_t averageDCEstimator(int32_t *p, uint16_t x) {
    *p += ((((long)x << 15) - *p) >> 4);
    return (*p >> 15);
  }

  //  Low Pass FIR Filter
  int16_t lowPassFIRFilter(int16_t din) {
    cbuf[offset] = din;

    int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

    for (uint8_t i = 0; i < 11; i++) {
      z += mul16(FIRCoeffs[i],
                 cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
    }

    offset++;
    offset %= 32; // Wrap condition

    return (z >> 15);
  }

  //  Integer multiplier
  int32_t mul16(int16_t x, int16_t y) { return ((long)x * (long)y); }

public:
  bool checkForBeat(int32_t sample) {

    //  Save current state
    IR_AC_Signal_Previous = IR_AC_Signal_Current;

    // This is good to view for debugging
    // Serial.print("Signal_Current: ");
    // Serial.println(IR_AC_Signal_Current);

    //  Process next data sample
    IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
    IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

    //  Detect positive zero crossing (rising edge)
    if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0)) {

      IR_AC_Max = IR_AC_Signal_max; // Adjust our AC max and min
      IR_AC_Min = IR_AC_Signal_min;

      positiveEdge = 1;
      negativeEdge = 0;
      IR_AC_Signal_max = 0;

      // if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
      if ((IR_AC_Max - IR_AC_Min) > 20 & (IR_AC_Max - IR_AC_Min) < 1000) {
        // Heart beat!!!
        beatDetected = true;
      }
    }

    //  Detect negative zero crossing (falling edge)
    if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0)) {
      positiveEdge = 0;
      negativeEdge = 1;
      IR_AC_Signal_min = 0;
    }

    //  Find Maximum value in positive cycle
    if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous)) {
      IR_AC_Signal_max = IR_AC_Signal_Current;
    }

    //  Find Minimum value in negative cycle
    if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous)) {
      IR_AC_Signal_min = IR_AC_Signal_Current;
    }

    return (beatDetected);
  }
};

void heartratecalc() {
  HeartRate *heartrate = (HeartRate *)malloc(sizeof(HeartRate));
  particleSensor = (MAX30105 *)malloc(sizeof(MAX30105));
  byte *rates = (byte *)malloc(sizeof(byte) * RATE_SIZE);
  // Initialize sensor
  if (heartrate == NULL || rates == NULL) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    displayError();
  }

  if (!particleSensor->begin(
          Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    displayError();
  }
  // Serial.println("Place your index finger on the sensor with steady
  // pressure.");

  particleSensor->setup(); // Configure sensor with default settings
  particleSensor->setPulseAmplitudeRed(
      0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor->setPulseAmplitudeGreen(0); // Turn off Green LED
  byte rateSpot = 0;
  long lastBeat = 0; // Time at which the last beat occurred
  float beatsPerMinute;
  int beatAvg;
  for (int i = 0; i < 3; ++i) {
    long irValue = particleSensor->getIR();

    if (heartrate->checkForBeat(irValue) == true) {
      // We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] =
            (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE;    // Wrap variable

        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    // Serial.print("IR=");
    // Serial.print(irValue);
    // Serial.print(", BPM=");
    // Serial.print(beatsPerMinute);
    // Serial.print(", Avg BPM=");
    // Serial.print(beatAvg);

    if (irValue < 50000)
      displayError();
  }
  correct_heartRate = beatAvg;
  digitalWrite(MAXLED, LOW);
  free(particleSensor);
  free(rates);
  free(heartrate);
}

void setup() {
  Serial.begin(9600);
  pinMode(MAXBTN, INPUT_PULLUP);
  pinMode(MAXLED, OUTPUT);
  pinMode(ECGLED, OUTPUT);
  pinMode(ECGBTN, INPUT_PULLUP);
}

void loop() {
  spo2calc();
  //  heartratecalc();
  ecgSetup();
  ecgDataCollect();
  ecgCleanup();
}

void displayError() {
  while (1) {
    digitalWrite(MAXLED, HIGH);
    digitalWrite(ECGLED, LOW);
    delay(500);
    digitalWrite(ECGLED, HIGH);
    digitalWrite(MAXLED, LOW);
    delay(500);
  }
}