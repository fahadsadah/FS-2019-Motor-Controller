#include <TimerOne/TimerOne.h>
#include <Arduino.h>

#define currentSensor 0
#define apps1 1
#define apps2 2

#define readyToDriveSwitch 2
#define tractivePowerSensing 3
#define brakeLightSwitch 4
#define motorOutput 5
#define speaker 6
#define readyToDriveLight 7

volatile int soundTimer = 0;
volatile bool readyToDrive = false;
volatile bool flash = false;
bool plausible = false;

void readyToDriveChange() {
    if (digitalRead(readyToDriveSwitch) == LOW) {
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

void lostTractivePower() {
    analogWrite(motorOutput, 0); //cut the power
    digitalWrite(speaker, LOW); //cancel the ready sound

    readyToDrive = false;
    digitalWrite(readyToDriveLight, LOW);
}

void timer() {
    if (soundTimer > 0) {
        soundTimer = soundTimer - 1;
        if (soundTimer == 0) {
            digitalWrite(speaker, LOW);
        }
    }

    if (!plausible && readyToDrive) {
        if (flash) {
            digitalWrite(readyToDriveLight, HIGH);
            flash = false;
        } else {
            digitalWrite(readyToDriveLight, LOW);
            flash = true;
        }
    }
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(analogInputToDigitalPin(currentSensor), INPUT);
    pinMode(analogInputToDigitalPin(apps1), INPUT);
    pinMode(analogInputToDigitalPin(apps2), INPUT);

    pinMode(readyToDriveSwitch, INPUT);
    pinMode(readyToDriveLight, OUTPUT);
    pinMode(brakeLightSwitch, INPUT);
    pinMode(motorOutput, OUTPUT);
    pinMode(speaker, OUTPUT);
    pinMode(tractivePowerSensing, INPUT);

    digitalWrite(brakeLightSwitch, LOW);
    digitalWrite(readyToDriveSwitch, LOW);

    TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz

    attachInterrupt(digitalPinToInterrupt(readyToDriveSwitch), readyToDriveChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(tractivePowerSensing), lostTractivePower, FALLING);

    Timer1.initialize(1500000l);
    Timer1.attachInterrupt(timer);

    Serial.println("Hello");
}

bool plausibilityCheck(int pot1, int pot2) {
    float pc1 = ((float) pot1 - 160) / 28.7;
    float pc2 = (float) pot2 / 24.3;

    float ratio = pc1 / pc2;

    return (ratio > 0.9) && (ratio < 1.1);
}

void loop() {
    // put your main code here, to run repeatedly:

    float V_1 = ((1 / 5) * 1023);
    float V_1075 = ((1.075 / 5) * 1023);
    float V_25 = ((2.5 / 5) * 1023);
    float V_3 = ((3 / 5) * 1023);

    int currentSens = analogRead(currentSensor); //Read current sensor voltage from pin 0
    int accelPot1 = analogRead(apps1); //Read pedal pot 1 voltage
    int accelPot2 = analogRead(apps2); //Read pedal pot 2 voltage
    int powerReq = 0; //Set power requested to 0

    plausible = plausibilityCheck(accelPot1, accelPot2);
    if (readyToDrive && plausible) {
        float accelPosition = accelPot2 * 0.136612021857924;
        float currentSensPerc = (((currentSens - V_25) / V_1075) * 100); //Calculate current draw %
        float powerReq1 = accelPosition - currentSensPerc;
        powerReq = map(powerReq1, 0, 100, 184, 654); //0.9V to 3.2V

        analogWrite(motorOutput, powerReq / 4);
    } else {
        analogWrite(motorOutput, 0);
    }

    if (!plausible) {
        Serial.println("Implausible!");
        Serial.print(accelPot1); Serial.print(" "); Serial.print(accelPot2);
    }
}
