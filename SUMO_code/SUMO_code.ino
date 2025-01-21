#include <QTRSensors.h>
// Motor left and right Pin Definitions(pwm & dir)
const int RPwm = 11, 
          RDir = 13, 
          LPwm = 3, 
          LDir = 12;

//Led pin status
const int ArduLed = 8;

//QTR sensor pins
QTRSensors qtr;

const uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];
const int Redge = A0, Ledge = A1;

//Ir proximity sensor
const int LSens = A2, //left
          RSens = A4, //rigth
          MSens = A3; //middle

// btn pin         
const int Button = 10;

//DIPStart switch
const int DS1 = 5, 
          DS2 = 6, 
          DS3 = 7;

// Constants
const int edgeThreshold = 100, 
          MaxSpeed = 160, 
          normalSpeed = 90, 
          TurnSpeed = 200, 
          edgeAvoidanceSpeed = 180;

const unsigned long edgeAvoidanceDuration =10, 
                    debounceTime = 50, 
                    actionInterval = 100;

const int T_before_start = 2000;

// Variables
unsigned long lastEdgeMillis = 0, 
              lastButtonPressTime = 0, 
              lastActionTime = 0;

bool robotStarted = false, avoidingEdge = false;

int leftEdge = 0, 
    rightEdge = 0, 
    
    leftProxi = 0, 
    middleProxi = 0, 
    rightProxi = 0;

enum ActionState { WAITING, ATTACK, TURN_LEFT, TURN_RIGHT, SEARCH_TARGET };

ActionState currentActionState = WAITING;

// Motor Control Functions
void setMotors(int leftSpeed, int rightSpeed) {
    digitalWrite(LDir, leftSpeed > 0);
    digitalWrite(RDir, rightSpeed > 0);
    analogWrite(LPwm, abs(leftSpeed));
    analogWrite(RPwm, abs(rightSpeed));
}

void checkSensors() {
  qtr.read(sensorValues);
    leftEdge = sensorValues[0];
    rightEdge = sensorValues[1];
    leftProxi = digitalRead(LSens);
    middleProxi = digitalRead(MSens);
    rightProxi = digitalRead(RSens);

    if (middleProxi == HIGH && leftEdge > edgeThreshold && rightEdge > edgeThreshold) currentActionState = ATTACK;
    else if(leftProxi == HIGH && leftEdge > edgeThreshold && rightEdge > edgeThreshold) currentActionState = TURN_LEFT;
    else if (rightProxi == HIGH && leftEdge > edgeThreshold && rightEdge > edgeThreshold) currentActionState = TURN_RIGHT;
    else currentActionState = SEARCH_TARGET;

    /* Serial.print(leftEdge );
    Serial.print(" ");
    Serial.print(rightEdge);
    Serial.print(" ");
    Serial.print(leftProxi);
    Serial.print(" ");
    Serial.print(middleProxi);
    Serial.print(" ");
    Serial.println(rightProxi);
    delay(50); */

}

void searchTarget() {
    if (avoidingEdge) {
        if (millis() - lastEdgeMillis < edgeAvoidanceDuration) {
            leftEdge < edgeThreshold ? setMotors(edgeAvoidanceSpeed, -edgeAvoidanceSpeed) : setMotors(-edgeAvoidanceSpeed, edgeAvoidanceSpeed);
        } else avoidingEdge = false;
    } else {
        if (leftEdge < edgeThreshold || rightEdge < edgeThreshold) {
            setMotors(-edgeAvoidanceSpeed, -edgeAvoidanceSpeed);//move backward
            lastEdgeMillis = millis();
            avoidingEdge = true;
        } else setMotors(normalSpeed, normalSpeed); //since no edge detect, move freely
    }
}

void performAction() {
    if (millis() - lastActionTime < actionInterval) return;
    lastActionTime = millis();
    
    switch (currentActionState) {
        case ATTACK:       setMotors(MaxSpeed, MaxSpeed); break;
        case TURN_LEFT:    setMotors(-TurnSpeed, TurnSpeed); break;
        case TURN_RIGHT:   setMotors(TurnSpeed, -TurnSpeed); break;
        case SEARCH_TARGET: searchTarget(); break;
        case WAITING:      setMotors(0, 0); break;
    }
}

void startupSequence() {
    digitalWrite(ArduLed, (millis() / 500) % 2 == 0);
}

void setup() {
  Serial.begin(9600);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){Ledge, Redge}, SensorCount);//Left, Right

    pinMode(RPwm, OUTPUT); pinMode(RDir, OUTPUT); pinMode(LPwm, OUTPUT); pinMode(LDir, OUTPUT);
    pinMode(ArduLed, OUTPUT); pinMode(Redge, INPUT); pinMode(Ledge, INPUT);
    pinMode(LSens, INPUT); pinMode(RSens, INPUT); pinMode(MSens, INPUT);
    pinMode(Button, INPUT_PULLUP);

  pinMode(DS1, INPUT_PULLUP);     // Set DS1 as an input with pull-up resistor
  pinMode(DS2, INPUT_PULLUP);     // Set DS2 as an input with pull-up resistor
  pinMode(DS3, INPUT_PULLUP);     // Set DS3 as an input with pull-up resistor

    while (digitalRead(Button) == HIGH) digitalWrite(ArduLed, HIGH);
    digitalWrite(ArduLed, LOW);
}

void loop() {
  /* checkSensors();
  searchTarget(); */
    if (!robotStarted) {
        if (digitalRead(Button) == LOW && (millis() - lastButtonPressTime >= debounceTime)) {
            delay(T_before_start);
            lastButtonPressTime = millis();
            robotStarted = true;
            currentActionState = SEARCH_TARGET;
        } else startupSequence();
    } else {
        checkSensors();
        performAction();
    }
}