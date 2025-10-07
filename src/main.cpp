#include "robot_command.h"
#include "radio_manager.h"
#include "configs.h"
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/as5047/MagneticSensorAS5047.h"

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

RadioManager radioManager(RADIO_CE, RADIO_CSN, RADIO_MOSI,
                          RADIO_MISO, RADIO_SCK, RADIO_IRQ, thisSlaveAddress);

BLDCMotor motor1 = BLDCMotor(11); 
BLDCMotor motor2 = BLDCMotor(11); 
BLDCMotor motor3 = BLDCMotor(11); 
BLDCMotor motor4 = BLDCMotor(11); 

BLDCDriver3PWM driver1 = BLDCDriver3PWM(PWM1_A, PWM1_B, PWM1_C, PWM1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PWM2_A, PWM2_B, PWM3_C, PWM4_EN);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PWM3_A, PWM3_B, PWM3_C, PWM4_EN);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(PWM4_A, PWM4_B, PWM3_C, PWM4_EN);

SPIClass sensor_spi(ENCODER_MOSI, ENCODER_MISO, ENCODER_SCK);

MagneticSensorAS5047 encoder1(ENCODER_CS1);
MagneticSensorAS5047 encoder2(ENCODER_CS2);
MagneticSensorAS5047 encoder3(ENCODER_CS3);
MagneticSensorAS5047 encoder4(ENCODER_CS4);

const float WHEEL_ANGLES[4] = {
    M_PI * (1.0 / 6.0), 
    M_PI * (5.0 / 6.0), 
    M_PI * (5.0 / 4.0), 
    M_PI * (7.0 / 4.0)
};

float jacobian[4][3];

void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    
    encoder1.init(&sensor_spi);
    encoder2.init(&sensor_spi);
    encoder3.init(&sensor_spi);
    encoder4.init(&sensor_spi);

    motor1.linkSensor(&encoder1);
    motor2.linkSensor(&encoder2);
    motor3.linkSensor(&encoder3);
    motor4.linkSensor(&encoder4);
    
    for (int i = 0; i < 4; i++) {
        jacobian[i][0] = sin(WHEEL_ANGLES[i]);
        jacobian[i][1] = cos(WHEEL_ANGLES[i]);
        jacobian[i][2] = ROBOT_RADIUS;
    }
    
    radioManager.init();
    
    driver1.voltage_power_supply = 18.5;
    driver2.voltage_power_supply = 18.5;
    driver3.voltage_power_supply = 18.5;
    driver4.voltage_power_supply = 18.5;
    
    driver1.voltage_limit = 12;
    driver2.voltage_limit = 12;
    driver3.voltage_limit = 12;
    driver4.voltage_limit = 12;

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

    motor1.PID_velocity.P = 0.2f;
    motor1.PID_velocity.I = 20;
    motor1.PID_velocity.D = 0;
    motor1.PID_velocity.output_ramp = 1000;

    motor2.PID_velocity.P = 0.2f;
    motor2.PID_velocity.I = 20;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 1000;

    motor3.PID_velocity.P = 0.2f;
    motor3.PID_velocity.I = 20;
    motor3.PID_velocity.D = 0;
    motor3.PID_velocity.output_ramp = 1000;

    motor4.PID_velocity.P = 0.2f;
    motor4.PID_velocity.I = 20;
    motor4.PID_velocity.D = 0;
    motor4.PID_velocity.output_ramp = 1000;
    
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

void calculateWheelSpeeds(float vx, float vy, float vt, float* result) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0.0;
        result[i] = result[i] + jacobian[i][0] * vx;
        result[i] = result[i] + jacobian[i][1] * vy;
        result[i] = result[i] + jacobian[i][2] * vt;
        result[i] = (1.0 / WHEEL_RADIUS) * result[i];
    }
}

static uint8_t msg_count = 0;
static uint8_t led_state = LOW;

void loop() {
    RobotCommand cmd;
    
    motor1.loopFOC();
    motor1.move(wheel_speeds[0]);

    motor2.loopFOC();
    motor2.move(wheel_speeds[1]);

    motor3.loopFOC();
    motor3.move(wheel_speeds[2]);

    motor4.loopFOC();
    motor4.move(wheel_speeds[3]);

    if(msg_count >= LED_COUNT_BLINK){

        if(led_state == LOW){
        led_state = HIGH;
        }
        else {
        led_state = LOW;
        }

        digitalWrite(LED_BUILTIN, led_state);
        msg_count = 0;
    }

    if (radioManager.checkAndReceive(cmd)) {
        printCommand(cmd);

        float vx = cmd.vx_linear / 1000.0f;  // Convert to appropriate units
        float vy = cmd.vy_linear / 1000.0f;  // Convert to appropriate units
        float vt = cmd.angular_speed / 1000.0f;  // Convert to appropriate units
        
        calculateWheelSpeeds(vx, vy, vt, wheel_speeds);

        msg_count++;
    }
}