
/*
  AQUORA - FULL INTEGRATED MONITORING SYSTEM
  Arduino Uno R3

  SENSORS:
    SEN0189 turbidity AO       -> A3
    YF-S401 flow signal        -> D2 (external interrupt)
    MC-38 reed switch          -> D3
    DS18B20 temperature DATA   -> D4
    HC-SR04 TRIG               -> D10
    HC-SR04 ECHO               -> D11

  OUTPUTS:
    RGB LED R/G/B              -> D5 / D6 / D9
    Passive buzzer driver      -> D7
    LCD1602 I2C SDA/SCL        -> A4 / A5

  74HC595:
    DATA                       -> A0
    CLOCK                      -> A1
    LATCH                      -> A2
    Q0-Q4                      -> Five blue level LEDs
    Q5                         -> Green cover CLOSED LED
    Q6                         -> Red cover OPEN LED
    Q7                         -> Reserved

  LCD PAGES (5 seconds each):
    1. Flow + Temperature
    2. Flow + Water Level
    3. Flow + Cover OPEN/CLOSED
    4. Flow + Turbidity

  BUZZER:
    2500 Hz
    10 seconds ON
    1 second OFF
    Repeats while an alarm condition exists.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


// ==================================================
// 1. LCD CONFIGURATION
// ==================================================

const byte LCD_ADDRESS = 0x27;

LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);


// ==================================================
// 2. PIN ASSIGNMENTS
// ==================================================

const byte TURBIDITY_PIN = A3;

const byte FLOW_PIN = 2;
const byte REED_PIN = 3;
const byte TEMP_PIN = 4;

const byte RGB_RED_PIN = 5;
const byte RGB_GREEN_PIN = 6;
const byte BUZZER_PIN = 7;
const byte RGB_BLUE_PIN = 9;

const byte ULTRASONIC_TRIG_PIN = 10;
const byte ULTRASONIC_ECHO_PIN = 11;

const byte SHIFT_DATA_PIN = A0;
const byte SHIFT_CLOCK_PIN = A1;
const byte SHIFT_LATCH_PIN = A2;


// ==================================================
// 3. 74HC595 OUTPUT ASSIGNMENTS
// ==================================================

const byte BIT_LEVEL_LED_1 = 0; // Q0
const byte BIT_LEVEL_LED_2 = 1; // Q1
const byte BIT_LEVEL_LED_3 = 2; // Q2
const byte BIT_LEVEL_LED_4 = 3; // Q3
const byte BIT_LEVEL_LED_5 = 4; // Q4

const byte BIT_COVER_GREEN = 5; // Q5: GREEN CLOSED
const byte BIT_COVER_RED = 6;   // Q6: RED OPEN

// Q7 reserved for future use.


// ==================================================
// 4. CALIBRATION SETTINGS
// ==================================================

// RGB LED
// false = common cathode
// true = common anode

const bool RGB_COMMON_ANODE = false;


// TURBIDITY THRESHOLDS

const float GREEN_THRESHOLD = 3.80;
const float YELLOW_THRESHOLD = 3.30;
const float ORANGE_THRESHOLD = 2.80;


// WATER LEVEL

const float EMPTY_DISTANCE_CM = 14.5;
const float FULL_DISTANCE_CM = 2.5;


// COVER SWITCH

const bool COVER_CLOSED_WHEN_LOW = true;

const unsigned long REED_DEBOUNCE_MS = 30;


// FLOW SENSOR

// Initial factor for DFRobot SEN0216.
// Calibrate your specific YF-S401 if necessary.

const float FLOW_PULSES_PER_LITER = 5880.0;


// BUZZER

// BUZZER
const unsigned int BUZZER_FREQUENCY_HZ = 2500;

// RED alarm: Existing behavior
const unsigned long BUZZER_ON_MS = 10000;
const unsigned long BUZZER_OFF_MS = 1000;

// ORANGE alarm: New behavior
const unsigned long ORANGE_BUZZER_ON_MS = 2000;
const unsigned long ORANGE_BUZZER_OFF_MS = 1000;
// ==================================================
// 5. TIMING SETTINGS
// ==================================================

const unsigned long FLOW_UPDATE_MS = 500;

const unsigned long TURBIDITY_UPDATE_MS = 500;

const unsigned long LEVEL_UPDATE_MS = 300;

const unsigned long TEMP_REQUEST_MS = 1000;

const unsigned long TEMP_CONVERSION_MS = 200;

const unsigned long LCD_PAGE_MS = 5000;

const unsigned long LCD_REFRESH_MS = 250;

const unsigned long SERIAL_REPORT_MS = 1000;

const unsigned long ULTRASONIC_TIMEOUT_US = 25000UL;


// ==================================================
// 6. SENSOR OBJECTS
// ==================================================

OneWire oneWire(TEMP_PIN);

DallasTemperature temperatureSensor(&oneWire);


// ==================================================
// 7. GLOBAL VARIABLES
// ==================================================


// FLOW SENSOR

volatile unsigned long flowPulseWindow = 0;

volatile unsigned long flowPulseTotal = 0;

float flowRateMLMin = 0.0;

float totalLiters = 0.0;

unsigned long lastFlowUpdateAt = 0;


// TURBIDITY

int turbidityADC = 0;

float turbidityVoltage = 0.0;

byte turbidityStatus = 0;

// 0 = GREEN
// 1 = YELLOW
// 2 = ORANGE
// 3 = RED

bool turbidityValid = false;

unsigned long lastTurbidityUpdateAt = 0;


// WATER LEVEL

float distanceCM = -1.0;

float waterLevelExact = 0.0;

int waterLevelPercent = 0;

int ledCount = 0;

bool waterLevelValid = false;

unsigned long lastLevelUpdateAt = 0;

unsigned long lastGoodLevelAt = 0;


// COVER

bool coverClosed = false;

bool coverCandidate = false;

unsigned long coverCandidateSince = 0;


// TEMPERATURE

float waterTemperatureC = 0.0;

bool temperatureValid = false;

bool temperatureConversionPending = false;

unsigned long temperatureRequestStartedAt = 0;

unsigned long lastTemperatureRequestAt = 0;


// SHIFT REGISTER

byte lastShiftRegisterValue = 0xFF;


// LCD

byte currentLCDPage = 0;

unsigned long lastLCDPageChangeAt = 0;

unsigned long lastLCDRefreshAt = 0;


// BUZZER
// BUZZER

bool buzzerAlarmWasActive = false;

bool buzzerToneIsOn = false;

unsigned long buzzerPhaseStartedAt = 0;

byte previousBuzzerMode = 0;

// 0 = OFF
// 1 = ORANGE
// 2 = RED / COVER OPEN

// SERIAL

unsigned long lastSerialReportAt = 0;


// ==================================================
// 8. FLOW SENSOR INTERRUPT
// ==================================================

void flowPulseISR() {

  flowPulseWindow++;

  flowPulseTotal++;

}


// ==================================================
// 9. RGB LED CONTROL
// ==================================================

void setRGB(
  byte redValue,
  byte greenValue,
  byte blueValue
) {

  if (RGB_COMMON_ANODE) {

    redValue = 255 - redValue;

    greenValue = 255 - greenValue;

    blueValue = 255 - blueValue;

  }

  analogWrite(RGB_RED_PIN, redValue);

  analogWrite(RGB_GREEN_PIN, greenValue);

  analogWrite(RGB_BLUE_PIN, blueValue);

}


// ==================================================
// 10. TURBIDITY STATUS TEXT
// ==================================================

const char* getTurbidityText() {

  switch (turbidityStatus) {

    case 0:
      return "GREEN";

    case 1:
      return "YELLOW";

    case 2:
      return "ORANGE";

    default:
      return "RED";

  }

}


// ==================================================
// 11. TURBIDITY SENSOR + RGB
// ==================================================

void updateTurbidity() {

  unsigned long total = 0;

  const byte samples = 10;


  for (byte i = 0; i < samples; i++) {

    total += analogRead(TURBIDITY_PIN);

  }


  turbidityADC = total / samples;

  turbidityVoltage =
    turbidityADC * (5.0 / 1023.0);

  turbidityValid = true;


  // LOWER VOLTAGE = HIGHER TURBIDITY

  if (turbidityVoltage >= GREEN_THRESHOLD) {

    turbidityStatus = 0;

    setRGB(0, 255, 0);

  }

  else if (turbidityVoltage >= YELLOW_THRESHOLD) {

    turbidityStatus = 1;

    setRGB(255, 190, 0);

  }

  else if (turbidityVoltage >= ORANGE_THRESHOLD) {

    turbidityStatus = 2;

    setRGB(255, 75, 0);

  }

  else {

    turbidityStatus = 3;

    setRGB(255, 0, 0);

  }

}


// ==================================================
// 12. FLOW SENSOR CALCULATION
// ==================================================

void updateFlowSensor(unsigned long now) {

  unsigned long elapsed = now - lastFlowUpdateAt;

  if (elapsed < FLOW_UPDATE_MS) {

    return;

  }


  unsigned long pulses;

  noInterrupts();

  pulses = flowPulseWindow;

  flowPulseWindow = 0;

  interrupts();


  if (
    FLOW_PULSES_PER_LITER > 0.0 &&
    elapsed > 0
  ) {

float litersThisWindow =
  (float)pulses / FLOW_PULSES_PER_LITER;

// Convert liters to milliliters.
float millilitersThisWindow =
  litersThisWindow * 1000.0;

// Calculate flow rate in mL/min.
flowRateMLMin =
  millilitersThisWindow *
  (60000.0 / (float)elapsed);

// Keep total accumulated volume in liters.
totalLiters += litersThisWindow;

  }


  lastFlowUpdateAt = now;

}


// ==================================================
// 13. HC-SR04 DISTANCE + WATER LEVEL
// ==================================================

float readUltrasonicDistanceCm() {

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  delayMicroseconds(2);

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);

  delayMicroseconds(10);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);


  unsigned long duration = pulseIn(
    ULTRASONIC_ECHO_PIN,
    HIGH,
    ULTRASONIC_TIMEOUT_US
  );


  if (duration == 0) {

    return -1.0;

  }


  return (duration * 0.0343) / 2.0;

}


void updateWaterLevel() {

  float measuredDistance =
    readUltrasonicDistanceCm();


  if (
    measuredDistance < 2.0 ||
    measuredDistance > 400.0
  ) {

    if (millis() - lastGoodLevelAt > 2000UL) {

      waterLevelValid = false;

      ledCount = 0;

    }

    return;

  }


  distanceCM = measuredDistance;


  if (EMPTY_DISTANCE_CM <= FULL_DISTANCE_CM) {

    waterLevelValid = false;

    return;

  }


  // Calculate percentage.

  waterLevelExact =

    ((EMPTY_DISTANCE_CM - distanceCM) /

    (EMPTY_DISTANCE_CM - FULL_DISTANCE_CM))

    * 100.0;


  // Prevent negative percentage.

  if (waterLevelExact < 0.0) {

    waterLevelExact = 0.0;

  }


  // Maximum 100%.

  if (waterLevelExact > 100.0) {

    waterLevelExact = 100.0;

  }


  waterLevelPercent =
    (int)(waterLevelExact + 0.5);


  waterLevelValid = true;

  lastGoodLevelAt = millis();


  // Round UP for five 20% LED sections.

  if (waterLevelExact <= 0.0) {

    ledCount = 0;

  }

  else {

    ledCount = (int)ceil(
      waterLevelExact / 20.0
    );

  }


  if (ledCount < 0) ledCount = 0;

  if (ledCount > 5) ledCount = 5;

}


// ==================================================
// 14. MC-38 COVER SWITCH
// ==================================================

bool readCoverState() {

  int reading = digitalRead(REED_PIN);


  if (COVER_CLOSED_WHEN_LOW) {

    return (reading == LOW);

  }

  return (reading == HIGH);

}


void setupCoverSwitch() {

  pinMode(REED_PIN, INPUT_PULLUP);

  coverClosed = readCoverState();

  coverCandidate = coverClosed;

  coverCandidateSince = millis();

}


void updateCoverSwitch(unsigned long now) {

  bool reading = readCoverState();


  if (reading != coverCandidate) {

    coverCandidate = reading;

    coverCandidateSince = now;

  }


  if (
    now - coverCandidateSince >=
    REED_DEBOUNCE_MS
  ) {

    coverClosed = coverCandidate;

  }

}


// ==================================================
// 15. DS18B20 TEMPERATURE SENSOR
// ==================================================

void startTemperatureConversion(unsigned long now) {

  temperatureSensor.requestTemperatures();

  temperatureConversionPending = true;

  temperatureRequestStartedAt = now;

  lastTemperatureRequestAt = now;

}


void updateTemperature(unsigned long now) {

  // Read completed conversion.

  if (
    temperatureConversionPending &&
    now - temperatureRequestStartedAt >=
    TEMP_CONVERSION_MS
  ) {

    float reading =
      temperatureSensor.getTempCByIndex(0);


    if (
      reading != DEVICE_DISCONNECTED_C &&
      !isnan(reading) &&
      reading >= -55.0 &&
      reading <= 125.0
    ) {

      waterTemperatureC = reading;

      temperatureValid = true;

    }

    else {

      temperatureValid = false;

    }


    temperatureConversionPending = false;

  }


  // Start next conversion.

  if (
    !temperatureConversionPending &&
    now - lastTemperatureRequestAt >=
    TEMP_REQUEST_MS
  ) {

    startTemperatureConversion(now);

  }

}


// ==================================================
// 16. 74HC595 SHIFT REGISTER
// ==================================================

void writeShiftRegister(byte outputValue) {

  if (outputValue == lastShiftRegisterValue) {

    return;

  }


  digitalWrite(SHIFT_LATCH_PIN, LOW);

  shiftOut(
    SHIFT_DATA_PIN,
    SHIFT_CLOCK_PIN,
    MSBFIRST,
    outputValue
  );

  digitalWrite(SHIFT_LATCH_PIN, HIGH);


  lastShiftRegisterValue = outputValue;

}


// ==================================================
// 17. ALL INDICATOR LEDs
// ==================================================

void updateAllLEDs() {

  byte outputPattern = 0;


  // FIVE BLUE WATER LEVEL LEDs

  if (waterLevelValid) {

    for (byte i = 0; i < (byte)ledCount; i++) {

      outputPattern |= (1 << i);

    }

  }


  // GREEN = CLOSED
  // RED = OPEN

  if (coverClosed) {

    outputPattern |= (1 << BIT_COVER_GREEN);

  }

  else {

    outputPattern |= (1 << BIT_COVER_RED);

  }


  // Q7 remains reserved.

  writeShiftRegister(outputPattern);

}


// ==================================================
// 18. PASSIVE BUZZER CONTROL
// ==================================================

void updateBuzzer(unsigned long now) {

  // RED alarm: Cover OPEN or turbidity RED.
  bool redAlarm =
    (!coverClosed) ||
    (turbidityValid && turbidityStatus == 3);

  // ORANGE alarm: Turbidity ORANGE.
  bool orangeAlarm =
    (turbidityValid && turbidityStatus == 2);

  // Determine alarm priority.
  byte currentMode = 0;

  if (redAlarm) {
    currentMode = 2;
  }
  else if (orangeAlarm) {
    currentMode = 1;
  }

  // Stop buzzer when no alarm exists.
  if (currentMode == 0) {

    if (buzzerToneIsOn) {
      noTone(BUZZER_PIN);
    }

    buzzerToneIsOn = false;
    buzzerAlarmWasActive = false;
    previousBuzzerMode = 0;

    return;
  }

  // Start immediately when alarm mode changes.
  if (currentMode != previousBuzzerMode) {

    previousBuzzerMode = currentMode;
    buzzerAlarmWasActive = true;

    buzzerToneIsOn = true;
    buzzerPhaseStartedAt = now;

    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);

    return;
  }

  // Select timing based on alarm mode.
  unsigned long onDuration =
    (currentMode == 2)
    ? BUZZER_ON_MS
    : ORANGE_BUZZER_ON_MS;

  unsigned long offDuration =
    (currentMode == 2)
    ? BUZZER_OFF_MS
    : ORANGE_BUZZER_OFF_MS;

  // Stop buzzer after ON duration.
  if (
    buzzerToneIsOn &&
    now - buzzerPhaseStartedAt >= onDuration
  ) {

    noTone(BUZZER_PIN);

    buzzerToneIsOn = false;
    buzzerPhaseStartedAt = now;
  }

  // Restart buzzer after OFF duration.
  else if (
    !buzzerToneIsOn &&
    now - buzzerPhaseStartedAt >= offDuration
  ) {

    tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);

    buzzerToneIsOn = true;
    buzzerPhaseStartedAt = now;
  }
}


// ==================================================
// 19. LCD DISPLAY
// ==================================================

void printLCDLine(
  byte row,
  const char* text
) {

  char buffer[17];

  size_t length = strlen(text);


  for (byte i = 0; i < 16; i++) {

    buffer[i] =
      (i < length) ? text[i] : ' ';

  }

  buffer[16] = '\0';


  lcd.setCursor(0, row);

  lcd.print(buffer);

}


// FLOW LINE

void buildFlowLine(char* line) {

  char flowText[10];

  dtostrf(
    flowRateMLMin,
    0,
    0,
    flowText
  );

  snprintf(
    line,
    17,
    "Flow:%s mL/min",
    flowText
  );

}


// FOUR LCD PAGES, FIVE SECONDS EACH

void updateLCD(unsigned long now) {

  if (now - lastLCDPageChangeAt >= LCD_PAGE_MS) {

    currentLCDPage =
      (currentLCDPage + 1) % 4;

    lastLCDPageChangeAt = now;

  }


  if (now - lastLCDRefreshAt < LCD_REFRESH_MS) {

    return;

  }


  char line1[17];

  char line2[17];


  buildFlowLine(line1);


  switch (currentLCDPage) {


    // --------------------------------------
    // PAGE 1: FLOW + TEMPERATURE
    // --------------------------------------

    case 0:

      if (temperatureValid) {

        char tempText[10];

        dtostrf(
          waterTemperatureC,
          0,
          1,
          tempText
        );


        snprintf(
          line2,
          sizeof(line2),
          "Temp: %s C",
          tempText
        );

      }

      else {

        snprintf(
          line2,
          sizeof(line2),
          "Temp: NO SENSOR"
        );

      }

      break;


    // --------------------------------------
    // PAGE 2: FLOW + WATER LEVEL
    // --------------------------------------

    case 1:

      if (waterLevelValid) {

        snprintf(
          line2,
          sizeof(line2),
          "Level: %d%%",
          waterLevelPercent
        );

      }

      else {

        snprintf(
          line2,
          sizeof(line2),
          "Level: NO ECHO"
        );

      }

      break;


    // --------------------------------------
    // PAGE 3: FLOW + STORAGE COVER
    // --------------------------------------

    case 2:

      snprintf(
        line2,
        sizeof(line2),
        "Cover: %s",
        coverClosed ? "CLOSED" : "OPEN"
      );

      break;


    // --------------------------------------
    // PAGE 4: FLOW + TURBIDITY
    // --------------------------------------

    default:

      if (turbidityValid) {

        snprintf(
          line2,
          sizeof(line2),
          "Turb: %s",
          getTurbidityText()
        );

      }

      else {

        snprintf(
          line2,
          sizeof(line2),
          "Turb: NO DATA"
        );

      }

      break;

  }


  printLCDLine(0, line1);

  printLCDLine(1, line2);


  lastLCDRefreshAt = now;

}


// ==================================================
// 20. SERIAL MONITOR
// ==================================================

void printSerialStatus(unsigned long now) {

  if (now - lastSerialReportAt < SERIAL_REPORT_MS) {

    return;

  }

  lastSerialReportAt = now;


  unsigned long pulseSnapshot;

  noInterrupts();

  pulseSnapshot = flowPulseTotal;

  interrupts();


  Serial.print(F("FLOW: "));

  Serial.print(flowRateMLMin, 2);

  Serial.print(F(" mL/min | TOTAL: "));

  Serial.print(totalLiters, 3);

  Serial.print(F(" L | PULSES: "));

  Serial.print(pulseSnapshot);


  Serial.print(F(" | TEMP: "));

  if (temperatureValid) {

    Serial.print(waterTemperatureC, 1);

    Serial.print(F(" C"));

  }

  else {

    Serial.print(F("NO SENSOR"));

  }


  Serial.print(F(" | LEVEL: "));

  if (waterLevelValid) {

    Serial.print(waterLevelPercent);

    Serial.print(F("% ("));

    Serial.print(distanceCM, 1);

    Serial.print(F(" cm)"));

  }

  else {

    Serial.print(F("NO ECHO"));

  }


  Serial.print(F(" | COVER: "));

  Serial.print(
    coverClosed ? F("CLOSED") : F("OPEN")
  );


  Serial.print(F(" | TURBIDITY: "));

  if (turbidityValid) {

    Serial.print(turbidityVoltage, 2);

    Serial.print(F(" V / "));

    Serial.print(getTurbidityText());

  }

  else {

    Serial.print(F("NO DATA"));

  }


  Serial.print(F(" | BUZZER: "));

  Serial.println(
    buzzerAlarmWasActive ? F("ALARM") : F("OFF")
  );

}


// ==================================================
// 21. SETUP
// ==================================================

void setup() {

  Serial.begin(9600);


  // LCD INITIALIZATION

  lcd.init();

  lcd.backlight();

  lcd.clear();

  printLCDLine(0, "AQUORA SYSTEM");

  printLCDLine(1, "Starting...");


  // TURBIDITY

  pinMode(TURBIDITY_PIN, INPUT);


  // RGB LED

  pinMode(RGB_RED_PIN, OUTPUT);

  pinMode(RGB_GREEN_PIN, OUTPUT);

  pinMode(RGB_BLUE_PIN, OUTPUT);

  setRGB(0, 0, 0);


  // FLOW SENSOR

  pinMode(FLOW_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(FLOW_PIN),
    flowPulseISR,
    RISING
  );


  // REED SWITCH

  setupCoverSwitch();


  // TEMPERATURE SENSOR

  temperatureSensor.begin();

  temperatureSensor.setResolution(10);

  temperatureSensor.setWaitForConversion(false);

  startTemperatureConversion(millis());


  // ULTRASONIC SENSOR

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);

  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);


  // 74HC595

  pinMode(SHIFT_DATA_PIN, OUTPUT);

  pinMode(SHIFT_CLOCK_PIN, OUTPUT);

  pinMode(SHIFT_LATCH_PIN, OUTPUT);


  digitalWrite(SHIFT_DATA_PIN, LOW);

  digitalWrite(SHIFT_CLOCK_PIN, LOW);

  digitalWrite(SHIFT_LATCH_PIN, HIGH);


  digitalWrite(SHIFT_LATCH_PIN, LOW);

  shiftOut(
    SHIFT_DATA_PIN,
    SHIFT_CLOCK_PIN,
    MSBFIRST,
    0
  );

  digitalWrite(SHIFT_LATCH_PIN, HIGH);

  lastShiftRegisterValue = 0;


  // PASSIVE BUZZER

  pinMode(BUZZER_PIN, OUTPUT);

  noTone(BUZZER_PIN);

  digitalWrite(BUZZER_PIN, LOW);


  // INITIAL TIMERS

  unsigned long now = millis();

  lastFlowUpdateAt = now;

  lastTurbidityUpdateAt = now;

  lastLevelUpdateAt = now;

  lastLCDPageChangeAt = now;

  lastLCDRefreshAt = 0;

  lastSerialReportAt = now;


  // INITIAL SENSOR READINGS

  updateTurbidity();

  updateWaterLevel();

  updateAllLEDs();

  updateLCD(millis());


  Serial.println(F("========================"));

  Serial.println(F("AQUORA FULL SYSTEM STARTED"));

  Serial.println(F("FLOW + TEMP + LEVEL"));

  Serial.println(F("COVER + TURBIDITY"));

  Serial.println(F("BUZZER ALERT ENABLED"));

  Serial.println(F("========================"));

}


// ==================================================
// 22. MAIN LOOP
// ==================================================

void loop() {

  unsigned long now = millis();


  // FLOW SENSOR
  // Pulse counting continues through D2 interrupt.

  updateFlowSensor(now);


  // TURBIDITY

  if (
    now - lastTurbidityUpdateAt >=
    TURBIDITY_UPDATE_MS
  ) {

    updateTurbidity();

    lastTurbidityUpdateAt = now;

  }


  // WATER LEVEL

  if (
    now - lastLevelUpdateAt >=
    LEVEL_UPDATE_MS
  ) {

    updateWaterLevel();

    lastLevelUpdateAt = now;

  }


  // STORAGE COVER

  updateCoverSwitch(now);


  // TEMPERATURE

  updateTemperature(now);


  // INDICATOR LEDS

  updateAllLEDs();


  // BUZZER

  updateBuzzer(now);


  // LCD

  updateLCD(now);


  // SERIAL MONITOR

  printSerialStatus(now);

}
