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

BLDCMotor motor1 = BLDCMotor(MOTOR_PP); 
BLDCMotor motor2 = BLDCMotor(MOTOR_PP); 
BLDCMotor motor3 = BLDCMotor(MOTOR_PP); 
BLDCMotor motor4 = BLDCMotor(MOTOR_PP);
BLDCMotor motors[4]{BLDCMotor(MOTOR_PP)};

BLDCDriver3PWM driver1 = BLDCDriver3PWM(PWM1_A, PWM1_B, PWM1_C, PWM1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PWM2_A, PWM2_B, PWM3_C, PWM4_EN);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PWM3_A, PWM3_B, PWM3_C, PWM4_EN);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(PWM4_A, PWM4_B, PWM3_C, PWM4_EN);

SPIClass sensor_spi(ENCODER_MOSI, ENCODER_MISO, ENCODER_SCK);

MagneticSensorAS5047 encoder1(ENCODER_CS1);
MagneticSensorAS5047 encoder2(ENCODER_CS2);
MagneticSensorAS5047 encoder3(ENCODER_CS3);
MagneticSensorAS5047 encoder4(ENCODER_CS4);

InverseKinematics ik;

void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    
    radioManager.init();
    
    encoder1.init(&sensor_spi);
    encoder2.init(&sensor_spi);
    encoder3.init(&sensor_spi);
    encoder4.init(&sensor_spi);

    motor1.linkSensor(&encoder1);
    motor2.linkSensor(&encoder2);
    motor3.linkSensor(&encoder3);
    motor4.linkSensor(&encoder4);
    
    
    driver1.voltage_power_supply = SUPPLY_VOLTAGE;
    driver2.voltage_power_supply = SUPPLY_VOLTAGE;
    driver3.voltage_power_supply = SUPPLY_VOLTAGE;
    driver4.voltage_power_supply = SUPPLY_VOLTAGE;
    
    driver1.voltage_limit = SUPPLY_LIMIT;
    driver2.voltage_limit = SUPPLY_LIMIT;
    driver3.voltage_limit = SUPPLY_LIMIT;
    driver4.voltage_limit = SUPPLY_LIMIT;

    driver1.init();
    driver2.init();
    driver3.init();
    driver4.init();
    
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);
    motor3.linkDriver(&driver3);
    motor4.linkDriver(&driver4);
    
    motor1.controller = MotionControlType::velocity;
    motor2.controller = MotionControlType::velocity;
    motor3.controller = MotionControlType::velocity;
    motor4.controller = MotionControlType::velocity;

    motor1.PID_velocity.P           = DEFAULT_KP;
    motor1.PID_velocity.I           = DEFAULT_KI;
    motor1.PID_velocity.D           = DEFAULT_KD;
    motor1.PID_velocity.output_ramp = VEL_RAMP;
    motor1.LPF_velocity.Tf          = DEFAULT_LPF;

    motor2.PID_velocity.P           = DEFAULT_KP;
    motor2.PID_velocity.I           = DEFAULT_KI;
    motor2.PID_velocity.D           = DEFAULT_KD;
    motor2.PID_velocity.output_ramp = VEL_RAMP;
    motor2.LPF_velocity.Tf          = DEFAULT_LPF;

    motor3.PID_velocity.P           = DEFAULT_KP;
    motor3.PID_velocity.I           = DEFAULT_KI;
    motor3.PID_velocity.D           = DEFAULT_KD;
    motor3.PID_velocity.output_ramp = VEL_RAMP;
    motor3.LPF_velocity.Tf          = DEFAULT_LPF;

    motor4.PID_velocity.P           = DEFAULT_KP;
    motor4.PID_velocity.I           = DEFAULT_KI;
    motor4.PID_velocity.D           = DEFAULT_KD;
    motor4.PID_velocity.output_ramp = VEL_RAMP;
    motor4.LPF_velocity.Tf          = DEFAULT_LPF;
    
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();
    
    motor1.initFOC();
    motor2.initFOC();
    motor3.initFOC();
    motor4.initFOC();
}

float wheel_speeds[4] = {0.0, 0.0, 0.0, 0.0};

static uint8_t msg_count = 0;
static uint8_t led_state = LOW;

void loop() {
    RobotCommand cmd;
    if(radioManager.getSignal(wheel_speeds))
    {
        motor1.loopFOC();
        motor1.move(wheel_speeds[0]);

        motor2.loopFOC();
        motor2.move(wheel_speeds[1]);

        motor3.loopFOC();
        motor3.move(wheel_speeds[2]);

        motor4.loopFOC();
        motor4.move(wheel_speeds[3]);
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