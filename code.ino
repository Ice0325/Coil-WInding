
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonStartPin = 6;
const int buttonStopPin = 7;
const int buttonIncreasePin = 8;
const int buttonDecreasePin = 9;
const int hallSensorPin = 11;

int max = 55.58;
double output = 20;
const int potentiometerPin = A0;
unsigned long lastDebounceTime = 0;

const int motorPin = 10;

double kp = 0.15;
double ki = 0.01; 
double kd = 0; 

double lastError = 0;        
double integral = 0;         
long lastTime = 0;
int wireCount = 0;
int targetWireCount = 10;
int desiredRpm = 0;

bool motorRunning = false;

unsigned long lastPressTime = 0;
bool increaseHeld = false;
bool decreaseHeld = false;

bool hallSensorState = HIGH;
bool hallSensorLastState = HIGH;

// PID variables
double setpoint, input;
// double Kp = 1.0, integral = 1.0, Kd = 1.0;
// PID myPID(&input, &output, &setpoint, Kp, integral, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

 pinMode(buttonStartPin, INPUT_PULLUP);
  pinMode(buttonStopPin, INPUT_PULLUP);
  pinMode(buttonIncreasePin, INPUT_PULLUP);
  pinMode(buttonDecreasePin, INPUT_PULLUP);

  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);


 updateDisplay();
}

void loop() {
  handleButtons();
  handlePotentiometer();
  handleHallSensor();
  if(motorRunning){
   pid();
  }
  Serial.print("Val1:");
  Serial.println(output);
  Serial.print("Val2:");

  Serial.println(desiredRpm);

}

void handleButtons() {
  if (digitalRead(buttonStartPin) == LOW && wireCount<targetWireCount) {
    lastDebounceTime=millis();
    motorRunning = true;

    analogWrite(motorPin, 255);
    delay(200);
  }

  if (digitalRead(buttonStopPin) == LOW) {
    motorRunning = false;
    analogWrite(motorPin, 0); 
 
    delay(200);
  }

  if (digitalRead(buttonIncreasePin) == LOW) {
    if (!increaseHeld) {
      increaseHeld = true;
      lastPressTime = millis();
      targetWireCount++;

    } else {
      if (millis() - lastPressTime > 1000) {
        targetWireCount++;
    
        delay(200);
      }
    }
  } else {
    increaseHeld = false;
  }

  if (digitalRead(buttonDecreasePin) == LOW) {
    if (!decreaseHeld) {
      decreaseHeld = true;
      lastPressTime = millis();
      if (targetWireCount > 0) targetWireCount--;

    } else {
      if (millis() - lastPressTime > 1000) {
        if (targetWireCount > 0) targetWireCount--;

        delay(200);
      }
    }
  } else {
    decreaseHeld = false;
  }

  updateDisplay();
}

void handlePotentiometer() {
  if(motorRunning){
  int potValue = analogRead(potentiometerPin);
  desiredRpm = map(potValue,0,1007,0,max);
  }
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wires: ");
  lcd.print(wireCount);
  lcd.print("/");
  lcd.print(targetWireCount);
  lcd.setCursor(0,1);
  int potValue = analogRead(potentiometerPin);
  lcd.print("Speed: ");
  lcd.print(String(output);
  lcd.print("/");
  lcd.print(desiredRpm);
  delay(50);
}





void handleHallSensor() {
  if(motorRunning){
  hallSensorState = digitalRead(hallSensorPin);
  
  if (hallSensorState == LOW && hallSensorLastState == HIGH) {
    wireCount++;
    updateDisplay();
  }
  if(wireCount>=targetWireCount){
    motorRunning=false;
    analogWrite(motorPin,0);
  }
  hallSensorLastState = hallSensorState;
  }
  
}



void pid() {

    double error = desiredRpm - output;
    integral += error * interval;
    double interval = (millis() - lastTime); 

    double derivative = (error - lastError) / interval;

    output = kp*error + ki*integral + kd*derivative;

    lastError = error;
    lastTime = millis();
    int pOutput = map(output, 0, max, 0, 255);

    if (pOutput > 255) {
        pOutput = 255;
    } 
    if (pOutput < 0) {
        pOutput = 0;
    }

    analogWrite(motorPin, pOutput);
   
}
