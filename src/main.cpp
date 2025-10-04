#include <Arduino.h>
#include "Wire.h"
#include "robot_command.h"
#include "radio_manager.h"
#include <SimpleFOC.h>
// #include "SimpleFOCDrivers.h"
// #include "encoders/as5047/MagneticSensorAS5047.h"


#define LED_COUNT_BLINK 20

#define RADIO_IRQ PD15
#define RADIO_CE PB10
#define RADIO_CSN PB11
#define RADIO_MOSI PE14
#define RADIO_MISO PE13
#define RADIO_SCK PE12

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};


RadioManager radioManager(RADIO_CE, RADIO_CSN, RADIO_MOSI, RADIO_MISO, RADIO_SCK, RADIO_IRQ, thisSlaveAddress);


BLDCMotor motor1 = BLDCMotor(11); 
BLDCMotor motor2 = BLDCMotor(11); 
BLDCMotor motor3 = BLDCMotor(11); 
BLDCMotor motor4 = BLDCMotor(11); 

BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA3, PC3_C);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA8, PA9, PA10, PA11);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PB6, PB7, PB8, PB9);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(PC6, PC7, PC8, PC9);

// SPIClass sensor_spi1(PB5, PB4, PB3);
// MagneticSensorSPI encoder1 = MagneticSensorSPI(PD6, 14, 0x3FFF);
// MagneticSensorSPI encoder2 = MagneticSensorSPI(AS5047_SPI, PD4);
// MagneticSensorAS5047 encoder2(PD4);

#define ROBOT_RADIUS 0.072f  // Robot radius in meters
#define WHEEL_RADIUS 0.034f  // Wheel radius in meters

const float WHEEL_ANGLES[4] = {
    M_PI * (1.0 / 6.0), 
    M_PI * (5.0 / 6.0), 
    M_PI * (5.0 / 4.0), 
    M_PI * (7.0 / 4.0)
};

float jacobian[4][3];

void setup() {    
    pinMode(PE3, OUTPUT);
    // Serial.begin(9600);

    // sensor_spi1.setClockDivider(128);

    // encoder1.init(&sensor_spi1);
    // encoder2.init(&sensor_spi1);

    // encoder1.min_elapsed_time = 0.01;
    // encoder2.min_elapsed_time = 0.001;
    
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

    // motor2.linkSensor(&encoder1);
    // motor3.linkSensor(&encoder2);
    
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);
    motor3.linkDriver(&driver3);
    motor4.linkDriver(&driver4);
    
    // motor2.controller = MotionControlType::velocity;
    // motor3.controller = MotionControlType::velocity;
    
    motor1.controller = MotionControlType::velocity_openloop;
    motor2.controller = MotionControlType::velocity_openloop;
    motor3.controller = MotionControlType::velocity_openloop;
    motor4.controller = MotionControlType::velocity_openloop;

    // motor2.PID_velocity.P = 0.2;
    // motor2.PID_velocity.I = 20;
    // motor2.PID_velocity.D = 0;
    // motor2.PID_velocity.output_ramp = 1000;
    // motor2.LPF_velocity.Tf = 0.01;

    // motor3.PID_velocity.P = 0.2;
    // motor3.PID_velocity.I = 20;
    // motor3.PID_velocity.D = 0;
    // motor3.PID_velocity.output_ramp = 1000;
    // motor3.LPF_velocity.Tf = 0.01;
    
    motor1.init();
    motor2.init();
    motor3.init();
    motor4.init();
    
    motor1.initFOC();
    motor2.initFOC();
    motor3.initFOC();
    motor4.initFOC();   

    // Serial.println("motor is ready");
    delay(100);
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

    // delay(100);
    
    motor1.loopFOC();
    motor1.move(wheel_speeds[0]);
    // motor1.move(target_velocity);

    motor2.loopFOC();
    motor2.move(wheel_speeds[1]);
    // motor2.move(target_velocity);

    motor3.loopFOC();
    motor3.move(wheel_speeds[2]);
    // motor3.move(target_velocity);

    motor4.loopFOC();
    motor4.move(wheel_speeds[3]);
    // motor4.move(target_velocity);

    // delay(100);

    // encoder2.update();

    // float a1 = encoder2.getAngle();

    // // get the velocity, in rad/s - note: you have to call getAngle() on a regular basis for it to work
    // float v1 = encoder2.getVelocity();

    // // get the angle, in radians, no full rotations
    // float a2 = encoder2.getCurrentAngle();

    // // get the raw 14 bit value
    // uint16_t raw = encoder2.readRawAngle();

    // // read the CORDIC magnitude value, a measure of the magnet field strength
    // float m1 = encoder2.readMagnitude();
    
    // Serial.print("Angle: ");
    // Serial.print(a1);
    // Serial.println(" ");

    // check for errors
    // if (encoder2.isErrorFlag()) {
    //     AS5047Error error = encoder2.clearErrorFlag();
    //     if (error.parityError) { // also error.framingError, error.commandInvalid
    //         Serial.println("parity error");   // etc...
    //     }
    //     if (error.framingError) { // also error.framingError, error.commandInvalid
    //         Serial.println("framing error");   // etc...
    //     }
    //     if (error.commandInvalid) { // also error.framingError, error.commandInvalid
    //         Serial.println("command invalid");   // etc...
    //     }
    // }

    // else{
        // Serial.print("Velocity: ");    
        // Serial.print(v1);
        // Serial.print(" ");
        // Serial.print("Current Angle: ");
        // Serial.print(a2);
        // Serial.print(" ");
        // Serial.print("Raw: ");
        // Serial.print(raw);
        // Serial.print(" ");
        // Serial.print("Mag: ");
        // Serial.print(m1);
        // Serial.println(" ");
    // }
    
    // Serial.print(" ");
    // Serial.print(encoder2.getVelocity());
    // Serial.print(" ");
    // Serial.print(encoder2.getSensorAngle());
    // Serial.print('\n');
    
    // delay(100);
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