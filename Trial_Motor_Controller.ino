#include <TimerOne.h>

#define currentSensor 0
#define apps1 1
#define apps2 2

#define readyToDriveSwitch 2
#define readyToDriveLight 3
#define brakeLightSwitch 4
#define motorOutput 5
#define speaker 6

int soundTimer = 0;
volatile bool readyToDrive = false;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);

    pinMode(readyToDriveSwitch, INPUT);
    pinMode(readyToDriveLight, OUTPUT);
    pinMode(brakeLightSwitch, INPUT);
    pinMode(motorOutput, OUTPUT);
    pinMode(speaker, OUTPUT);

    TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz

    attachInterrupt(digitalPinToInterrupt(2), checkReadyToDriveSwitch, CHANGE);

    Timer1.initialize(1500000);
    Timer1.attachInterrupt(timer);
}

void checkReadyToDriveSwitch() {
    if (digitalRead(readyToDriveSwitch) == LOW) { // RTD switched off
        analogWrite(motorOutput, 0); //cut the power
        digitalWrite(speaker, LOW); //cancel the ready sound

        readyToDrive = false;
        digitalWrite(readyToDriveLight, LOW);
    } else if (digitalRead(brakeLightSwitch) == HIGH) { // RTD switched on, and brake pedal pressed
        readyToDrive = true;
        digitalWrite(readyToDriveLight, HIGH);

        soundTimer = 2;
        digitalWrite(speaker, HIGH);
    }
}

void timer() {
    if (soundTimer == 0) {
        digitalWrite(speaker, LOW);
    } else {
        soundTimer--;
    }
}

void loop() {
    // put your main code here, to run repeatedly:

    float V_1 = ((1/5)*1023);
    float V_1075 = ((1.075/5)*1023);
    float V_25= ((2.5/5)*1023);
    float V_3 = ((3/5)*1023);

    int currentSens = analogRead(currentSensor); //Read current sensor voltage from pin 0
    int accelPot1 = analogRead(apps1); //Read pedal pot 1 voltage
    int accelPot2 = analogRead(apps2); //Read pedal pot 2 voltage
    int powerReq = 0; //Set power requested to 0

    float accelPot1Trav= ((accelPot1/1023.0)*100); //Calculate pedal pot 1 travel %
    float accelPot2Trav= ((accelPot2/1023.0)*100); //Calculate pedal pot 2 travel %
    float currentSensPerc = (((currentSens-V_25)/V_1075)*100); //Calculate current draw %

    float powerReq1 = accelPot1Trav-currentSensPerc;//Calculate total requested current
    powerReq = map(powerReq1, 0, 100, 184, 654); //0.9V to 3.2V

    if (readyToDrive) {
        analogWrite(motorOutput, powerReq / 4);
    }

    float vOut = powerReq*(5.0/1023);
    float ped = accelPot1*(5.0/1023);
    float sens = currentSens*(5.0/1023);

/*
  Serial.print("Accel % = ");
  Serial.println(ped);
  Serial.print("Current Sensor % = ");
  Serial.println(sens);
  Serial.print("Power Delivery = ");
  Serial.println(vOut);
 */

    Serial.print(ped);Serial.print(",");
    Serial.print(sens);Serial.print(",");
    Serial.print(vOut);Serial.println("");

    delay(2);
}
