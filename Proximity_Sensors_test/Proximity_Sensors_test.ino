const int LSens = A2;  // Left opponent sensor
const int RSens = A4;  // Right opponent sensor
const int MSens = A3;  // Middle opponent sensor

void setup() {
    Serial.begin(9600);         // Initialize serial communication
    pinMode(LSens, INPUT);       // Set LSens as input
    pinMode(RSens, INPUT);       // Set RSens as input
    pinMode(MSens, INPUT);       // Set MSens as input
}

void loop() {
    int lSensState = digitalRead(LSens);  // Read left opponent sensor
    int rSensState = digitalRead(RSens);  // Read right opponent sensor
    int mSensState = digitalRead(MSens);  // Read middle opponent sensor

    // Check each sensor's state and print if it detects an opponent
/*     if (lSensState == HIGH) {
        Serial.println("Opponent detected on the left!");
    } */
    if (rSensState == HIGH) {
        Serial.println("Opponent detected on the right!");
    }
/*     if (mSensState == HIGH) {
        Serial.println("Opponent detected in the middle!");
    } */

    delay(200);  // Small delay to avoid flooding the Serial Monitor
}
