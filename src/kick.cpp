#include "kick.h"
#include <Arduino.h>
Kick::Kick
(
    int kickPin,
    int voltageSensorPin,
    int infraPin,
    float kickVoltage
)
{
    this->kickPin = kickPin;
    this->voltageSensorPin = voltageSensorPin;
    this->infraPin = infraPin;
    this->kickVoltage = kickVoltage;
    pinMode(kickPin, OUTPUT);
    pinMode(voltageSensorPin, INPUT);
    pinMode(infraPin, INPUT);
}
float Kick::getVoltage()
{
    return (map(analogRead(voltageSensorPin), 0, 825, 0, 4000))/1000;
}
void Kick::makeKick()
{
    if(digitalRead(infraPin) == true && getVoltage() >= 2.3)
    {
        digitalWrite(kickPin, HIGH);
        delay(5);
        digitalWrite(kickPin, LOW);
    }
}