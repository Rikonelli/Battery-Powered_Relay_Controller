// Battery-powered Relay Controller Firmware
// For Arduino Uno with TP4056, MT3608, and 5V Relay Module
// Optimized for low power consumption with 18650 battery

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Pin Definitions
const int PUSH_BUTTON_PIN = 2;    // Push button input (must be pin 2 for INT0)
const int LED_PIN = 4;            // Status LED
const int RELAY_PIN = 7;          // Relay control
const int BATTERY_PIN = A0;       // Battery voltage monitoring
const int BUZZER_PIN = 10;        // Buzzer control

// Constants
const unsigned long LONG_PRESS_DURATION = 2000;  // Long press duration in milliseconds
const unsigned long BLINK_INTERVAL = 100;        // LED blink interval for low battery (ms)
const unsigned long BUZZER_SHORT_BEEP_DURATION = 200;  // Short buzzer beep duration (ms)
const unsigned long BUZZER_LONG_BEEP_DURATION = 500;   // Long buzzer beep duration (ms)
const float LOW_BATTERY_THRESHOLD = 3.3;         // Low battery threshold (Volts)
const float VOLTAGE_DIVIDER_RATIO = 1.0;         // No voltage divider used
const float REFERENCE_VOLTAGE = 5.0;             // Reference voltage for ADC
const unsigned long DEBOUNCE_DELAY = 50;         // Debounce time in milliseconds
const unsigned long BATTERY_CHECK_INTERVAL = 60000; // Check battery every minute (was 5 seconds)
const unsigned long BUZZER_LOW_BATTERY_INTERVAL = 900000; // 15 minutes between low battery beeps (was 5 minutes)

// Toggle intervals (milliseconds) - Only 1s and 10s
const unsigned long TOGGLE_INTERVALS[] = {1000, 10000};
const int NUM_INTERVALS = sizeof(TOGGLE_INTERVALS) / sizeof(TOGGLE_INTERVALS[0]);

// Variables
volatile bool deviceState = false;          // Device ON/OFF state
volatile bool relayState = false;           // Current relay state
volatile int currentIntervalIndex = 0;      // Current toggle frequency index
unsigned long lastToggleTime = 0;           // Last relay toggle time
volatile unsigned long buttonPressStartTime = 0; // Button press start time
volatile bool buttonPressed = false;        // Button pressed state
volatile bool longPressDetected = false;    // Flag for when long press is detected
bool lowBatteryState = false;               // Low battery state
unsigned long lastBlinkTime = 0;            // Last LED blink time for low battery or feedback
unsigned long lastBuzzerTime = 0;           // Last time buzzer was triggered
volatile unsigned long lastButtonChangeTime = 0; // Time of last button state change for debouncing
volatile bool lastButtonState = HIGH;       // Last stable button state
unsigned long lastBatteryCheckTime = 0;     // Last time battery was checked

// Sleep mode related
volatile bool wakeFlag = false;             // Flag to indicate wakeup from sleep

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println(F("Battery-powered Relay Controller - Optimized"));
  
  // Disable unused peripherals to save power (but keep necessary ones)
  power_adc_disable();      // Will enable when needed
  power_spi_disable();      // Disable SPI
  power_twi_disable();      // Disable I2C
  power_timer1_disable();   // Disable Timer 1
  power_timer2_disable();   // Disable Timer 2
  
  // Initialize pins
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);  // Push button with internal pull-up
  pinMode(LED_PIN, OUTPUT);                // LED output
  pinMode(RELAY_PIN, OUTPUT);              // Relay output
  pinMode(BATTERY_PIN, INPUT);             // Battery voltage monitoring
  pinMode(BUZZER_PIN, OUTPUT);             // Buzzer output
  
  // Initial states
  deviceState = false;      // Explicitly set device to OFF at startup
  relayState = false;       // Ensure relay state is initialized as OFF
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);           // Ensure buzzer is off initially
  
  // Use interrupt for button press - required for wake up from sleep
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), buttonInterrupt, CHANGE);
  
  // Disable watchdog timer at startup since device is OFF
  wdt_disable();
  
  Serial.println(F("System initialized - Device is OFF"));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check if we just woke up from sleep
  if (wakeFlag && !deviceState) {
    // If this is our first loop iteration after waking up
    // and if the button state is stable for a short time,
    // toggle the device ON
    if (digitalRead(PUSH_BUTTON_PIN) == HIGH && (currentTime - lastButtonChangeTime > DEBOUNCE_DELAY * 2)) {
      wakeFlag = false;
      toggleDevice();
      Serial.println(F("Device turned ON after wake from sleep"));
    }
  }

  // Check for long press while button is held down
  if (buttonPressed && !longPressDetected && (currentTime - buttonPressStartTime >= LONG_PRESS_DURATION)) {
    longPressDetected = true;
    
    // IMPORTANT: Only process long press if the device is already ON
    if (deviceState) {
      // Change toggle frequency
      changeToggleFrequency();
      
      // Force relay to OPEN state (HIGH = open, LOW = closed according to requirements)
      relayState = true;  // Set to true (open)
      digitalWrite(RELAY_PIN, HIGH);
      
      // Update LED to reflect relay state
      if (!lowBatteryState) {
        digitalWrite(LED_PIN, HIGH);  // LED HIGH when relay open
      }
      
      // Reset toggle timer to start new frequency immediately
      lastToggleTime = currentTime;
      
      // Visual feedback: Rapidly blink LED 3 times to indicate frequency change
      for (int i = 0; i < 3; i++) {
        // Blink LED
        digitalWrite(LED_PIN, LOW);
        
        // Wait a bit before changing the state
        delay(40);
        
        // Turn LED back on
        digitalWrite(LED_PIN, HIGH);
        
        // Wait again before changing the state
        delay(40);
      }
      
      Serial.println(F("Long press: changed frequency while device ON"));
    } else {
      // Device is OFF, so we don't change any relay state or provide feedback
      // Just mark the long press as detected so a short press isn't triggered when released
      Serial.println(F("Long press detected but device is OFF - no action taken"));
    }
  }
  
  // Check battery voltage periodically - only every minute instead of every 5 seconds
  if (currentTime - lastBatteryCheckTime >= BATTERY_CHECK_INTERVAL) {
    checkBattery();
    lastBatteryCheckTime = currentTime;
  }
  
  // Handle device state and relay toggling
  if (deviceState) {
    // Only toggle relay if device is ON
    toggleRelay();
    
    // Update LED based on relay state, unless low battery
    if (!lowBatteryState && !buttonPressed) {
      digitalWrite(LED_PIN, relayState ? HIGH : LOW);  // LED HIGH when relay open
    }
  }
  
  // Handle low battery indication (blink LED) and buzzer sound
  if (lowBatteryState) {
    handleLowBatteryIndication();
  }
  
  // Enter sleep mode if:
  // 1. Device is OFF (no need to toggle relay)
  // 2. No button is currently pressed
  // 3. Not in the middle of handling low battery indication
  if (!deviceState && !buttonPressed && (!lowBatteryState || (currentTime - lastBlinkTime) >= BLINK_INTERVAL)) {
    enterSleepMode();
  }
}

// Interrupt handler for button state changes
void buttonInterrupt() {
  // Set wake flag to indicate we've woken up from sleep
  bool wasAsleep = !deviceState && !buttonPressed;
  wakeFlag = true;
  
  unsigned long currentTime = millis();
  
  // Read the current button state
  bool currentButtonState = digitalRead(PUSH_BUTTON_PIN);
  
  // Check if enough time has passed since the last state change (debounce)
  if (currentTime - lastButtonChangeTime >= DEBOUNCE_DELAY) {
    // If the button state has genuinely changed (after debounce)
    if (currentButtonState != lastButtonState) {
      lastButtonState = currentButtonState;
      lastButtonChangeTime = currentTime;
      
      if (currentButtonState == LOW) {  // Button pressed (LOW because of pull-up)
        buttonPressed = true;
        longPressDetected = false;
        buttonPressStartTime = currentTime;
        Serial.println(F("Button pressed"));
        
        // If we were asleep (device off) and just woke up with this button press,
        // ONLY toggle the device when button is RELEASED, not on press
        // This avoids both short and long press toggling the device when it's off
        if (wasAsleep) {
          // Don't toggle device immediately - wait for release
          // We'll handle this in the release code
          Serial.println(F("Device woke up from sleep"));
        }
      }
      else if (currentButtonState == HIGH) {  // Button released
        // Only handle short press if long press wasn't already detected
        if (buttonPressed && !longPressDetected) {
          // Short press: Toggle device ON/OFF
          toggleDevice();
          Serial.println(F("Toggle device on short press"));
        }
        
        // Reset flags after button release
        buttonPressed = false;
        Serial.println(F("Button released"));
      }
    }
  }
}

// Toggle device ON/OFF
void toggleDevice() {
  deviceState = !deviceState;
  
  if (!deviceState) {
    // Turn off relay when device is turned off
    digitalWrite(RELAY_PIN, LOW);
    relayState = false;
    digitalWrite(LED_PIN, LOW);
    
    // Disable watchdog timer when device is OFF
    wdt_disable();
  } else {
    // Force relay to OPEN state initially when turning on
    digitalWrite(RELAY_PIN, HIGH);
    relayState = true;
    if (!lowBatteryState) {
      digitalWrite(LED_PIN, HIGH);  // LED HIGH when relay open
    }
    lastToggleTime = millis();  // Reset toggle timer
    
    // Configure watchdog timer when device is ON
    configureWatchdog();
  }
  
  Serial.print(F("Device: "));
  Serial.println(deviceState ? F("ON") : F("OFF"));
}

// Change the relay toggle frequency
void changeToggleFrequency() {
  // Toggle between the two intervals (1s and 10s)
  currentIntervalIndex = (currentIntervalIndex + 1) % NUM_INTERVALS;
  
  Serial.print(F("Toggle interval changed to: "));
  Serial.print(TOGGLE_INTERVALS[currentIntervalIndex] / 1000.0);
  Serial.println(F(" seconds"));
}

// Toggle relay based on current interval
void toggleRelay() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastToggleTime >= TOGGLE_INTERVALS[currentIntervalIndex]) {
    lastToggleTime = currentTime;
    relayState = !relayState;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    
    // Debug relay state
    Serial.print(F("Relay: "));
    Serial.println(relayState ? F("OPEN") : F("CLOSED"));
  }
}

// Check battery voltage and update low battery state
void checkBattery() {
  // Enable ADC before reading
  power_adc_enable();
  delay(1); // Brief delay for ADC to stabilize
  
  // Read battery voltage with multiple samples for better accuracy
  int batteryADC = 0;
  for(int i = 0; i < 3; i++) {
    batteryADC += analogRead(BATTERY_PIN);
    delay(1);
  }
  batteryADC /= 3; // Average of 3 readings
  
  float batteryVoltage = (batteryADC / 1023.0) * REFERENCE_VOLTAGE / VOLTAGE_DIVIDER_RATIO;
  
  // Disable ADC after reading to save power
  power_adc_disable();
  
  // Update low battery state
  lowBatteryState = (batteryVoltage < LOW_BATTERY_THRESHOLD);
  
  Serial.print(F("Battery Voltage: "));
  Serial.print(batteryVoltage);
  Serial.print(F("V - Status: "));
  Serial.println(lowBatteryState ? F("LOW") : F("OK"));
}

// Handle the low battery LED blinking and buzzer sound
void handleLowBatteryIndication() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    
    // Toggle LED for blinking effect
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Trigger buzzer beep for critical battery every 15 minutes
    if (currentTime - lastBuzzerTime >= BUZZER_LOW_BATTERY_INTERVAL) {
      tone(BUZZER_PIN, 1000);  // 1kHz tone
      delay(BUZZER_LONG_BEEP_DURATION);
      noTone(BUZZER_PIN);  // Turn off buzzer
      lastBuzzerTime = currentTime;
    }
  }
}

// Configure the watchdog timer
void configureWatchdog() {
  // Clear the reset flag
  MCUSR &= ~(1 << WDRF);
  
  // Enable the WDT change
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  
  // Set watchdog to interrupt mode with a 1-second timeout
  // WDP2, WDP1, WDP0 bits set the timeout (see datasheet)
  WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1); // ~1 second
}

// Watchdog interrupt service routine
ISR(WDT_vect) {
  // This is called when watchdog times out
  // No need to do anything, just wake up
}

// Enter sleep mode
void enterSleepMode() {
  // Disable serial to save power
  Serial.end();
  
  // Disable ADC before sleep
  ADCSRA &= ~(1 << ADEN);
  
  // Make sure the interrupt is properly set up before sleep
  // This is crucial - we need to ensure interrupt is cleared and ready
  attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_PIN), buttonInterrupt, CHANGE);
  
  // Configure sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  // Disable brown-out detection during sleep (saves more power)
  // BODS and BODSE bits need to be set in MCUCR
  MCUCR |= (1 << BODS) | (1 << BODSE);
  MCUCR &= ~(1 << BODSE); // Must be done right before sleep
  
  // Clear any pending interrupts before sleeping
  cli();
  wakeFlag = false;
  sei();
  
  // Sleep now
  sleep_cpu();
  
  // Program continues from here after wake-up
  sleep_disable();
  
  // Re-enable ADC
  ADCSRA |= (1 << ADEN);
  
  // Re-enable serial
  Serial.begin(9600);
  
  // Small delay to ensure system is fully awake
  delay(50);
  
  Serial.println(F("Woke up from sleep"));
  
  // Instead of trying to force a button handler here,
  // set a flag that will be checked in the loop
  wakeFlag = true;
}

// test