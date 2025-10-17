#include "configs.h"
#include "robot_command.h"
#include "radio_manager.h"
#include "inverse_kinematics.h"
#include <SimpleFOC.h>
#include "SimpleFOCDrivers.h"
#include "encoders/as5047/MagneticSensorAS5047.h"
#include "kick.h"

#define RADIO_IRQ  PD15
#define RADIO_CE   PB10
#define RADIO_CSN  PB11
#define RADIO_MOSI PE14
#define RADIO_MISO PE13
#define RADIO_SCK  PE12

#define ROBOT_ID 1

const byte thisSlaveAddress[5] = {'A','R','A','R','A'};

RadioManager radioManager(RADIO_CE, RADIO_CSN, RADIO_MOSI, RADIO_MISO, RADIO_SCK, RADIO_IRQ, thisSlaveAddress);

BLDCMotor motor1 = BLDCMotor(11); 
BLDCMotor motor2 = BLDCMotor(11); 
BLDCMotor motor3 = BLDCMotor(11); 
BLDCMotor motor4 = BLDCMotor(11); 

BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA0, PA1, PA3, PC3_C);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PC6, PC7, PC8, PC9);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PA8, PA9, PA10, PA11);
BLDCDriver3PWM driver4 = BLDCDriver3PWM(PB6, PB7, PB8, PB9);

SPIClass sensor_spi1(PA7, PA6, PA5);

MagneticSensorAS5047 encoder1(PA2);
MagneticSensorAS5047 encoder2(PA4);
MagneticSensorAS5047 encoder3(PD4);
MagneticSensorAS5047 encoder4(PD6);

#define ROBOT_RADIUS 0.072f  // Robot radius in meters
#define WHEEL_RADIUS 0.034f  // Wheel radius in meters

const float WHEEL_ANGLES[4] = {
    M_PI * (1.0 / 6.0), 
    M_PI * (5.0 / 6.0), 
    M_PI * (5.0 / 4.0), 
    M_PI * (7.0 / 4.0)
};

BLDCDriver3PWM drivers[4]
{
    BLDCDriver3PWM(PWM1_A, PWM1_B, PWM1_C, PWM1_EN),
    BLDCDriver3PWM(PWM2_A, PWM2_B, PWM3_C, PWM4_EN),
    BLDCDriver3PWM(PWM3_A, PWM3_B, PWM3_C, PWM4_EN),
    BLDCDriver3PWM(PWM4_A, PWM4_B, PWM3_C, PWM4_EN)
};

void setup() {    
    pinMode(PE3, OUTPUT);
    pinMode(PC10, INPUT);
    
    encoder1.init(&sensor_spi1);
    encoder2.init(&sensor_spi1);
    encoder3.init(&sensor_spi1);
    encoder4.init(&sensor_spi1);

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
    
    driver1.voltage_limit = 16;
    driver2.voltage_limit = 16;
    driver3.voltage_limit = 16;
    driver4.voltage_limit = 16;

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
    motor1.PID_velocity.I = 20.0f;
    motor1.PID_velocity.D = 0;
    motor1.PID_velocity.output_ramp = 1000;
    motor1.LPF_velocity.Tf = 0.01;

    motor2.PID_velocity.P = 0.2f;
    motor2.PID_velocity.I = 20.0f;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 1000;
    motor2.LPF_velocity.Tf = 0.01;

    motor3.PID_velocity.P = 0.2f;
    motor3.PID_velocity.I = 20.0f;
    motor3.PID_velocity.D = 0;
    motor3.PID_velocity.output_ramp = 1000;
    motor3.LPF_velocity.Tf = 0.01;

    motor4.PID_velocity.P = 0.2f;
    motor4.PID_velocity.I = 20.0f;
    motor4.PID_velocity.D = 0;
    motor4.PID_velocity.output_ramp = 1000;
    motor4.LPF_velocity.Tf = 0.01;
    
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

void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    
    radioManager.init();
    for(int i = 0; i < sizeof(motors); i++)
    {
        encoders[i].init(&sensor_spi);

        motors[i].linkSensor(&encoders[i]);
        
        drivers[i].voltage_power_supply = SUPPLY_VOLTAGE;
        drivers[i].voltage_limit        = SUPPLY_LIMIT;
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

static uint32_t nmsg_count = 0;

void loop() {
    RobotCommand cmd;

    motor2.loopFOC();
    motor2.move(wheel_speeds[1]);

    motor3.loopFOC();
    motor3.move(wheel_speeds[2]);

    motor4.loopFOC();
    motor4.move(wheel_speeds[3]);

    if(nmsg_count >= 2000){
        wheel_speeds[0] = 0.0;
        wheel_speeds[1] = 0.0;
        wheel_speeds[2] = 0.0;
        wheel_speeds[3] = 0.0;
        nmsg_count = 0;
        digitalWrite(LED_BUILTIN, LOW);
    }

    if (radioManager.checkAndReceive(cmd)) {
        if (cmd.id == ROBOT_ID){
            digitalWrite(LED_BUILTIN, HIGH);
            printCommand(cmd);
    
            float vx_world = cmd.vx_linear / 1000.0f;
            float vy_world = cmd.vy_linear / 1000.0f;
            float vt = cmd.angular_speed / 1000.0f;
            float robot_angle = cmd.angle / 1000.0f;
            
            // Transform world velocities to robot frame
            float cos_a = cos(robot_angle);
            float sin_a = sin(robot_angle);
            float vx_robot = vx_world * cos_a + vy_world * sin_a;
            float vy_robot = -vx_world * sin_a + vy_world * cos_a;
            
            calculateWheelSpeeds(vx_robot, vy_robot, vt, wheel_speeds);

            if(cmd.kick_front == true){
                if(digitalRead(PC10) == LOW){
                    digitalWrite(LED_BUILTIN, LOW);
                }
            }
        }
        else{
            nmsg_count++;
        }
    }
    else{
        nmsg_count++;
    }
}