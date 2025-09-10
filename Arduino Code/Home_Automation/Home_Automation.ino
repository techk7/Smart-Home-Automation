#include <IRremote.h>

#define IR_PIN 2  //IR Pin
#define SWITCH1_PIN 4 // Manual Switch1
#define SWITCH2_PIN 5 // Manual Switch2
#define SSR1_PIN 8    // Triac !
#define SSR2_PIN 9    // Triac 2
#define PIR_PIN 6     // PIR Oin
#define SSR_PIN 7     // Triac 3

IRrecv irrecv(IR_PIN);
decode_results results;

// Variables for appliance state
bool applianceState = false; // Current state of the appliance (ON/OFF)
bool manualOverride = false; // Tracks if IR remote manually controlled the appliance
unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 100; // Timeout for motion detection (100ms)

// Manual override variables
unsigned long manualOverrideDuration = 3000; // 3 seconds delay for PIR reactivation
unsigned long manualOverrideStartTime = 0;
bool relay1State = false;
bool relay2State = false;

// Debounce timing
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
const unsigned long debounceDelay = 50; // 50ms debounce time for switches

// IR debounce timing
unsigned long lastIRTime = 0;
const unsigned long IRDebounceDelay = 500; // 500ms debounce for IR signals

// Define IR codes  //Replace Your IR Code
#define IR_CODE_BUTTON_1 0x2C87261 // IR code for Button 1 (PIR control)     
#define IR_CODE_BUTTON_2 0xFC04E1DD // IR code for Button 2 (Relay 1 control)
#define IR_CODE_BUTTON_3 0x1644C1C1 // IR code for Button 3 (Relay 2 control)

void setup() {
  // Initialize pins
  pinMode(PIR_PIN, INPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(SWITCH1_PIN, INPUT_PULLUP); // Use internal pull-up for NC switch
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(SSR1_PIN, OUTPUT);
  pinMode(SSR2_PIN, OUTPUT);

  // Initialize SSR states
  digitalWrite(SSR_PIN, LOW);
  digitalWrite(SSR1_PIN, LOW);
  digitalWrite(SSR2_PIN, LOW);

  irrecv.enableIRIn(); // Start the IR receiver
  Serial.begin(9600);
  Serial.println("Home Automation System Initialized");
}

void loop() {
  handlePIR();     // Handle PIR motion detection
  handleIR();      // Handle IR remote commands
  handleSwitches(); // Handle manual switch inputs
  handleTimeout(); // Handle timeout for motion detection
}

void handlePIR() {
  if (digitalRead(PIR_PIN) == HIGH && !manualOverride) {
    if (!applianceState) {
      applianceState = true;
      digitalWrite(SSR_PIN, HIGH);
      Serial.println("PIR Motion Detected: Appliance turned ON");
    }
    lastMotionTime = millis();
  }
}

void handleIR() {
  if (irrecv.decode(&results)) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastIRTime > IRDebounceDelay) { // IR debounce
      Serial.print("IR Code Received: ");
      Serial.println(results.value, HEX);

      if (results.value == IR_CODE_BUTTON_1) { // Handle Button 1 (PIR control)
        if (!applianceState) {  // If appliance is OFF, turn it ON
          applianceState = true;
          digitalWrite(SSR_PIN, HIGH); // Turn ON appliance
          Serial.println("Light1 turned ON via IR Button 1");
        } else {  // If appliance is ON, turn it OFF
          applianceState = false;
          digitalWrite(SSR_PIN, LOW);  // Turn OFF appliance
          Serial.println("Light1 turned OFF via IR Button 1");
        }
      } else if (results.value == IR_CODE_BUTTON_2) { // Handle Button 2 (Relay 1 control)
        // Toggle SSR1
        static bool ssr1State = false;
        ssr1State = !ssr1State;
        digitalWrite(SSR1_PIN, ssr1State ? HIGH : LOW);  // Toggle SSR1
        Serial.println(ssr1State ? "Fan turned ON via IR Button 2" : "Fan turned OFF via IR Button 2");
      } else if (results.value == IR_CODE_BUTTON_3) { // Handle Button 3 (Relay 2 control)
        // Ensure IR controls SSR2 independently
        static bool irControlSSR2 = false;
        irControlSSR2 = !irControlSSR2; // Toggle state on each IR button press
        digitalWrite(SSR2_PIN, irControlSSR2 ? HIGH : LOW); // Update SSR2 state
        Serial.println(irControlSSR2 ? "Light2 turned ON via IR Button 3" : "Light2 turned OFF via IR Button 3");
      }

      manualOverride = true; // Enable manual override
      manualOverrideStartTime = millis();
      lastIRTime = currentMillis; // Update IR debounce time
    }
    irrecv.resume();
  }
}


void handleSwitches() {
  // Handle Switch 1 (Normally Closed - NC)
  if (digitalRead(SWITCH1_PIN) == LOW) { // When switch is pressed (NC)
    if (millis() - lastDebounceTime1 > debounceDelay) {
      if (!relay1State) { // Only toggle if the state is different
        relay1State = true;  // Set relay to ON when switch is pressed
        digitalWrite(SSR1_PIN, HIGH); // Turn on SSR1
        Serial.println("Fan Switch turned ON (NC)");
      }
      lastDebounceTime1 = millis();
    }
  } else { // When switch is released (NC contact opens)
    if (millis() - lastDebounceTime1 > debounceDelay) {
      if (relay1State) { // Only toggle if the state is different
        relay1State = false; // Set relay to OFF when switch is released
        digitalWrite(SSR1_PIN, LOW); // Turn off SSR1
        Serial.println("Fan Switch turned OFF");
      }
      lastDebounceTime1 = millis();
    }
  }

  // Handle Switch 2 (Similar behavior as Switch 1)
  if (digitalRead(SWITCH2_PIN) == LOW) { // When switch is pressed (NC)
    if (millis() - lastDebounceTime2 > debounceDelay) {
      if (!relay2State) { // Only toggle if the state is different
        relay2State = true;  // Set relay to ON when switch is pressed
        digitalWrite(SSR2_PIN, HIGH); // Turn on SSR2
        Serial.println("Light2 Switch turned ON (NC)");
      }
      lastDebounceTime2 = millis();
    }
  } else { // When switch is released (NC contact opens)
    if (millis() - lastDebounceTime2 > debounceDelay) {
      if (relay2State) { // Only toggle if the state is different
        relay2State = false; // Set relay to OFF when switch is released
        digitalWrite(SSR2_PIN, LOW); // Turn off SSR2
        Serial.println("Light2 Switch turned OFF");
      }
      lastDebounceTime2 = millis();
    }
  }
}



void handleTimeout() {
  if (!manualOverride && applianceState && millis() - lastMotionTime > motionTimeout) {
    applianceState = false;
    digitalWrite(SSR_PIN, LOW);
    Serial.println("Timeout: Appliance turned OFF due to no motion");
  } 
  // Only reset manualOverride when the appliance is OFF
  else if (manualOverride && !applianceState && millis() - manualOverrideStartTime > manualOverrideDuration) {
    manualOverride = false;
    Serial.println("Manual Override ended: PIR reactivated");
  }
}

