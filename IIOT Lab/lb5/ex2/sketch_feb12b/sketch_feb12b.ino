#include <SPI.h>
#include <AIR430BoostETSI.h>

// Communication message definitions
unsigned char txDrunk[] = {'D', 'R', 'U', 'N', 'K', '_', 'C', 'A', 'R', '1', '2', '3', '\0'};  // Message for drunk state
unsigned char txSafe[] = {'S', 'A', 'F', 'E', '\0'};  // Message for safe state
unsigned char drunk_msg[] = { 0x30, 'A','E','S','I','O','T','D','B','M','N','R','\0' };  // Alternative drunk message format

// Pin definitions for buttons and LEDs
#define ok_btn PUSH1       // Safe driver button
#define drunk_btn PUSH2    // Drunk detection button
#define ok_led GREEN_LED   // Safe status LED
#define drunk_led RED_LED  // Drunk alert LED

// System state variables
bool carStarted = false;
bool breathTestDone = false;
bool isDrunk = false;
unsigned long sensorStartTime;
const int BREATH_TEST_TIME = 3000;  // 3 seconds for breath test

// Function to print drunk status message
void printTxDrunk() {
    Serial.print("TX (DATA): ");
    Serial.println((char*)txDrunk);
}

// Function to print safe status message
void printTxSafe() {
    Serial.print("TX (DATA): ");
    Serial.println((char*)txSafe);
}

// Function to handle drunk detection state
void handleDrunkState() {
    digitalWrite(ok_led, LOW);      // Turn off safe LED
    digitalWrite(drunk_led, HIGH);   // Turn on drunk LED
    Radio.transmit(ADDRESS_BROADCAST, txDrunk, sizeof(txDrunk));
    printTxDrunk();
    
    // Enter infinite loop for drunk detection
    while(1) {
        Radio.transmit(ADDRESS_BROADCAST, drunk_msg, 12);
        Serial.println((char*)drunk_msg);
        
        // Update message counter if within range
        if (drunk_msg[0] >= '0' && drunk_msg[0] < '9') {
            drunk_msg[0]++;
        } else {
            drunk_msg[0] = '0';  // Reset counter
        }
        
        delay(1000);  // Delay between transmissions
        Serial.println("SYSTEM LOCKED - Drunk driving detected");
    }
}

// Function to handle safe state
void handleSafeState() {
    digitalWrite(ok_led, HIGH);     // Turn on safe LED
    digitalWrite(drunk_led, LOW);    // Turn off drunk LED
    Radio.transmit(ADDRESS_BROADCAST, txSafe, sizeof(txSafe));
    printTxSafe();
}

void setup() {
    // Initialize radio communication
    Radio.begin(0x01, CHANNEL_1, POWER_MAX);
    Serial.begin(9600);
    
    // Configure pins
    pinMode(ok_led, OUTPUT);
    pinMode(drunk_led, OUTPUT);
    pinMode(ok_btn, INPUT_PULLUP);
    pinMode(drunk_btn, INPUT_PULLUP);
    
    // Initial state - both LEDs off
    digitalWrite(ok_led, LOW);
    digitalWrite(drunk_led, LOW);
    
    Serial.println("System initialized. Please perform breath test.");
    
    // Initial breath test loop
    unsigned int ok = !digitalRead(ok_btn);
    unsigned int drunk = !digitalRead(drunk_btn);
    
    while(1) {
        ok = !digitalRead(ok_btn);
        drunk = !digitalRead(drunk_btn);
        
        // Check for button press
        if(ok || drunk) {
            if(drunk) {
                handleDrunkState();  // Handle drunk detection
            } else if(ok) {
                // Start breath test timing
                if(!carStarted) {
                    sensorStartTime = millis();
                    carStarted = true;
                }
                
                // Check if breath test duration is met
                if(millis() - sensorStartTime >= BREATH_TEST_TIME) {
                    breathTestDone = true;
                    handleSafeState();  // Handle safe state
                    break;  // Exit setup and proceed to main loop
                }
            }
        } else {
            carStarted = false;  // Reset if button released too early
        }
        
        delay(100);  // Small delay to prevent excessive polling
    }
}

void loop() {
    // Continuous monitoring after initial setup
    if(breathTestDone) {
        // Check drunk button
        if(!digitalRead(drunk_btn)) {
            handleDrunkState();  // Enter drunk state if detected
        }
        
        // Regular safe status check
        if(!digitalRead(ok_btn)) {
            handleSafeState();  // Update safe status
        }
    }
    
    delay(100);  // Small delay for stability
}
