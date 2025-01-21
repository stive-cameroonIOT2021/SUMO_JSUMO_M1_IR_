// Motor control pins
const int RPwm = 11;
const int RDir = 13;
const int LPwm = 3;
const int LDir = 12;

void setup() {
    // Set motor pins as outputs
    pinMode(RPwm, OUTPUT);
    pinMode(RDir, OUTPUT);
    pinMode(LPwm, OUTPUT);
    pinMode(LDir, OUTPUT);

    // Initialize Serial for feedback
    Serial.begin(9600);
    Serial.println("Motor Test Start");
}

void loop() {
    // Test right motor forward
    digitalWrite(RDir, HIGH);         // Set right motor direction forward
    analogWrite(RPwm, 150);           // Set right motor speed
    Serial.println("Right Motor Forward");

      // Test left motor forward
    digitalWrite(LDir, HIGH);         // Set left motor direction forward
    analogWrite(LPwm, 150);           // Set left motor speed
    Serial.println("Left Motor Forward");
    /* delay(1000);                      // Run for 1 second

    // Test right motor backward
    digitalWrite(RDir, LOW);          // Set right motor direction backward
    analogWrite(RPwm, 150);           // Maintain speed
    Serial.println("Right Motor Backward");
    delay(1000);                      // Run for 1 second

    // Stop right motor
    analogWrite(RPwm, 0);
    Serial.println("Right Motor Stop");
    delay(500);                       // Wait for half a second

    // Test left motor forward
    digitalWrite(LDir, HIGH);         // Set left motor direction forward
    analogWrite(LPwm, 150);           // Set left motor speed
    Serial.println("Left Motor Forward");
    delay(1000);                      // Run for 1 second

    // Test left motor backward
    digitalWrite(LDir, LOW);          // Set left motor direction backward
    analogWrite(LPwm, 150);           // Maintain speed
    Serial.println("Left Motor Backward");
    delay(1000);                      // Run for 1 second

    // Stop left motor
    analogWrite(LPwm, 0);
    Serial.println("Left Motor Stop");
    delay(2000);                      // Wait before repeating the cycle */
}
