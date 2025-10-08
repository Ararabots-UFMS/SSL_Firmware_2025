#ifndef __KICK__
#define __KICK__
class Kick
{
    private:
        int kickPin;
        int voltageSensorPin;
        int infraPin;
        float kickVoltage;
        float getVoltage();
    public:
        Kick
        (
            int kickPin,
            int voltageSensorPin,
            int infraPin,
            float kickVoltage
        );
        void makeKick();
};
#endif