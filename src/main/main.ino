// Battery-powered Relay Controller Firmware
// For Arduino Uno with TP4056, MT3608, and 5V Relay Module
// Powered from a single 18650 battery
// Author: Wojciech Rykaczewski

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

const int PUSH_BUTTON_PIN = 2;
const int LED_PIN = 4;
const int RELAY_PIN = 7;
const int BATTERY_PIN = A0;

const unsigned long LONG_PRESS_DURATION = 2000;
const unsigned long BLINK_INTERVAL = 200;
const float LOW_BATTERY_THRESHOLD = 3.3;
const float REFERENCE_VOLTAGE = 5.0;
const float VOLTAGE_DIVIDER_RATIO = 1.0;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long BATTERY_CHECK_INTERVAL = 60000;
const unsigned long TOGGLE_INTERVALS[] = {1000, 10000};
const int NUM_INTERVALS = sizeof(TOGGLE_INTERVALS) / sizeof(TOGGLE_INTERVALS[0]);

volatile bool deviceState = false;
volatile bool relayState = false;
volatile int currentIntervalIndex = 0;
volatile unsigned long lastToggleTime = 0;
volatile unsigned long buttonPressStartTime = 0;
volatile bool buttonPressed = false;
volatile bool longPressDetected = false;
volatile bool lowBatteryState = false;
volatile unsigned long lastBlinkTime = 0;
volatile unsigned long lastButtonChangeTime = 0;
volatile bool lastButtonState = HIGH;
volatile unsigned long lastBatteryCheckTime = 0;
volatile bool wakeFlag = false;
volatile bool ledBlinkState = false;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Battery-powered Relay Controller"));

  // Disable unused peripherals to save power
  power_adc_disable();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
  power_timer2_disable();
  
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  
  deviceState = false;
  relayState = false;
  ledBlinkState = false;
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), buttonInterrupt, CHANGE);
  
  wdt_disable();
  
  checkBattery();
  
  Serial.println(F("System initialized - Device is OFF"));
}

void loop() {
  unsigned long currentTime = millis();
  
  if (wakeFlag && !deviceState) {
    if (digitalRead(PUSH_BUTTON_PIN) == HIGH && 
       (currentTime - lastButtonChangeTime > DEBOUNCE_DELAY * 3)) {
      wakeFlag = false;
      toggleDevice();
    }
  }

  if (buttonPressed && !longPressDetected && (currentTime - buttonPressStartTime >= LONG_PRESS_DURATION)) {
    longPressDetected = true;
    
    if (deviceState) {
      changeToggleFrequency();
      relayState = true;
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
      ledBlinkState = false;
      
      lastToggleTime = currentTime;
      
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(40);
        digitalWrite(LED_PIN, HIGH);
        delay(40);
      }
      
      Serial.println(F("Long press: changed frequency while device ON"));
    } else {
      Serial.println(F("Long press detected but device is OFF - no action taken"));
    }
  }
  
  if (currentTime - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL) {
    checkBattery();
    lastBatteryCheckTime = currentTime;
  }
  
  if (deviceState) {
    toggleRelay();
    
    if (relayState) {
      digitalWrite(LED_PIN, HIGH);
      ledBlinkState = false;
    } else if (!lowBatteryState) {
      digitalWrite(LED_PIN, LOW);
      ledBlinkState = false;
    }
  }
  
  if (lowBatteryState && deviceState && !relayState) {
    handleLowBatteryIndication();
  }
  
  if (!deviceState && !buttonPressed && (!lowBatteryState || (currentTime - lastBlinkTime) >= BLINK_INTERVAL)) {
    enterSleepMode();
  }
}

void buttonInterrupt() {
  wakeFlag = true;

  unsigned long currentTime = millis();
  bool currentButtonState = digitalRead(PUSH_BUTTON_PIN);

  if (currentTime - lastButtonChangeTime >= DEBOUNCE_DELAY) {
    if (currentButtonState != lastButtonState) {
      lastButtonState = currentButtonState;
      lastButtonChangeTime = currentTime;

      if (currentButtonState == LOW) {
        buttonPressed = true;
        longPressDetected = false;
        buttonPressStartTime = currentTime;
        Serial.println(F("Button pressed"));
      }
      else {
        if (buttonPressed && !longPressDetected) {
          if (deviceState) delay(DEBOUNCE_DELAY * 2);
          toggleDevice();
          Serial.println(F("Toggle device on short press"));
        }
        buttonPressed = false;
        Serial.println(F("Button released"));
      }
    }
  }
}

void toggleDevice() {
  deviceState = !deviceState;
  
  if (!deviceState) {
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    digitalWrite(LED_PIN, LOW);
    ledBlinkState = false;
    lowBatteryState = false;
    wdt_disable();
  } else {
    checkBattery();
    digitalWrite(RELAY_PIN, HIGH);
    relayState = true;
    digitalWrite(LED_PIN, HIGH);
    ledBlinkState = false;
    lastToggleTime = millis();
    configureWatchdog();
  }
  
  delay(DEBOUNCE_DELAY * 2);
  lastButtonChangeTime = millis();
  
  Serial.print(F("Device: "));
  Serial.println(deviceState ? F("ON") : F("OFF"));
}

void changeToggleFrequency() {
  currentIntervalIndex = (currentIntervalIndex + 1) % NUM_INTERVALS;
  
  Serial.print(F("Toggle interval changed to: "));
  Serial.print(TOGGLE_INTERVALS[currentIntervalIndex] / 1000.0);
  Serial.println(F(" seconds"));
}

void toggleRelay() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastToggleTime >= TOGGLE_INTERVALS[currentIntervalIndex]) {
    lastToggleTime = currentTime;
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    
    if (relayState) {
      digitalWrite(LED_PIN, HIGH);
      ledBlinkState = false;
    } else if (!lowBatteryState) {
      digitalWrite(LED_PIN, LOW);
      ledBlinkState = false;
    }
    
    Serial.print(F("Relay: "));
    Serial.println(relayState ? F("OPEN") : F("CLOSED"));
  }
}

void checkBattery() {
  power_adc_enable();
  delay(5);
  
  int batteryADC = 0;
  for(int i = 0; i < 3; i++) {
    batteryADC += analogRead(BATTERY_PIN);
    delay(2);
  }
  batteryADC /= 3;
  
  float batteryVoltage = (batteryADC / 1023.0) * REFERENCE_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
  
  power_adc_disable();
  
  bool previousLowBatteryState = lowBatteryState;
  lowBatteryState = (batteryVoltage < LOW_BATTERY_THRESHOLD);
  
  if (lowBatteryState && !previousLowBatteryState) {
    lastBlinkTime = millis() - BLINK_INTERVAL;
    
    if (deviceState && !relayState) {
      ledBlinkState = true;
      digitalWrite(LED_PIN, HIGH);
    }
  }
  
  Serial.print(F("Battery Voltage: "));
  Serial.print(batteryVoltage);
  Serial.print(F("V - Status: "));
  Serial.println(lowBatteryState ? F("LOW") : F("OK"));
}

void handleLowBatteryIndication() {
  if (!deviceState || relayState) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    ledBlinkState = !ledBlinkState;
    digitalWrite(LED_PIN, ledBlinkState);
  }
}

void configureWatchdog() {
  // Clears the WDRF (Watchdog Reset Flag) in MCUSR register
  // to prevent an infinite loop of resets
  MCUSR &= ~(1 << WDRF);
  
  // Sets WDCE (Watchdog Change Enable) and WDE (Watchdog Enable)
  // to allow changing watchdog settings within 4 clock cycles
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  
  // Configures Watchdog Timer:
  // WDIE: enables watchdog interrupt instead of reset
  // WDP2 | WDP1: sets timeout to approximately 1 second (0.5s)
  WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);
}

ISR(WDT_vect) {
  // Watchdog interrupt handler - TBD per use case and after more testing
}

void enterSleepMode() {
  Serial.println(F("Entering sleep mode..."));
  Serial.flush();
  Serial.end();
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Enables Brown-Out Detection Disable (BODS) and Brown-Out Detection Sampling Enable (BODSE)
  // Prepares to disable Brown-Out Detection during sleep mode to save power
  MCUCR |= (1 << BODS) | (1 << BODSE);
  
  // Clears BODSE while keeping BODS active, which disables Brown-Out Detection to save power
  // This sequence must be executed within 4 clock cycles of the previous instruction
  MCUCR &= ~(1 << BODSE);
  
  cli();
  wakeFlag = false;
  sei();
  
  sleep_cpu();
  
  sleep_disable();
  //ADCSRA |= (1 << ADEN);  //turning ADC off using bit manipulation, leaving it here just in case one wants to use the basic option at some point; same is achieved by power_adc_disable();
  Serial.begin(9600);
  delay(50);
  
  Serial.println(F("Woke up from sleep"));
  wakeFlag = true;
}
