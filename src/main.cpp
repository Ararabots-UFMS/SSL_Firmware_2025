#include "configs.h"
#include "robot_command.h"
#include "radio_manager.h"
#include "inverse_kinematics.h"
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

const byte thisSlaveAddress[5] = SLAVE_ADDR;

RadioManager radioManager(RADIO_CE, RADIO_CSN, RADIO_MOSI,
                          RADIO_MISO, RADIO_SCK, RADIO_IRQ, thisSlaveAddress);

SPIClass sensor_spi(ENCODER_MOSI, ENCODER_MISO, ENCODER_SCK);
                       

BLDCMotor motors[4]
{
    BLDCMotor(MOTOR_PP),
    BLDCMotor(MOTOR_PP),
    BLDCMotor(MOTOR_PP),
    BLDCMotor(MOTOR_PP)
};

BLDCDriver3PWM drivers[4]
{
    BLDCDriver3PWM(PWM1_A, PWM1_B, PWM1_C, PWM1_EN),
    BLDCDriver3PWM(PWM2_A, PWM2_B, PWM3_C, PWM4_EN),
    BLDCDriver3PWM(PWM3_A, PWM3_B, PWM3_C, PWM4_EN),
    BLDCDriver3PWM(PWM4_A, PWM4_B, PWM3_C, PWM4_EN)
};

MagneticSensorAS5047 encoders[4]
{
    MagneticSensorAS5047(ENCODER_CS1),
    MagneticSensorAS5047(ENCODER_CS2),
    MagneticSensorAS5047(ENCODER_CS3),
    MagneticSensorAS5047(ENCODER_CS4)
};

InverseKinematics ik;

void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    
    radioManager.init();
    for(int i = 0; i<4; i++)
    {
        encoders[i].init(&sensor_spi);

        motors[i].linkSensor(&encoders[0]);
        
        drivers[i].voltage_power_supply = SUPPLY_VOLTAGE;
        drivers[i].voltage_limit = SUPPLY_LIMIT;
        drivers[i].init();
        
        motors[i].linkDriver(&drivers[i]);
        
        motors[i].controller = MotionControlType::velocity;

        motors[i].PID_velocity.P           = DEFAULT_KP;
        motors[i].PID_velocity.I           = DEFAULT_KI;
        motors[i].PID_velocity.D           = DEFAULT_KD;
        motors[i].PID_velocity.output_ramp = VEL_RAMP;
        motors[i].LPF_velocity.Tf          = DEFAULT_LPF;

        motors[i].init();
        motors[i].initFOC();
    }
}

float wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

static uint8_t msg_count = 0;
static uint8_t led_state = LOW;

void loop() {
    RobotCommand cmd;
    if(radioManager.getSignal(wheel_speeds))
    {
        for(int i = 0; i < 4; i++)
        {
            motors[i].loopFOC();
            motors[i].move(wheel_speeds[i]);
        }
    }

    if(msg_count >= LED_COUNT_BLINK){
        led_state != led_state;
        digitalWrite(LED_BUILTIN, led_state);
        msg_count = 0;
    }

    if (radioManager.checkAndReceive(cmd)) {
        printCommand(cmd);

        float vx = cmd.vx_linear / 1000.0f;  // Convert to appropriate units
        float vy = cmd.vy_linear / 1000.0f;  // Convert to appropriate units
        float vt = cmd.angular_speed / 1000.0f;  // Convert to appropriate units
        
        ik.calculateWheelSpeeds(vx, vy, vt, wheel_speeds);

        msg_count++;
    }
}