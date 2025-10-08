#ifndef CONFIGS_H
#define CONFIGS_H

#include "math.h"

// -------------------------- PINS

// KICK
#define KICK_PIN 1
#define VOLTAGE_SENSOR_PIN 2
#define INFRA_PIN 3
#define KICK_VOLTAGE 220

// RADIO SPI
#define RADIO_IRQ       PD15
#define RADIO_CE        PB10
#define RADIO_CSN       PB11
#define RADIO_MOSI      PE14
#define RADIO_MISO      PE13
#define RADIO_SCK       PE12

// ENCODER SPI
#define ENCODER_MOSI    PB5
#define ENCODER_MISO    PB4
#define ENCODER_SCK     PB3
#define ENCODER_CS1     PD4
#define ENCODER_CS2     PD6
#define ENCODER_CS3     PA2
#define ENCODER_CS4     PA4

// MOTOR 1 PWM
#define PWM1_A          PA0
#define PWM1_B          PA1
#define PWM1_C          PA3
#define PWM1_EN         PC3_C

// MOTOR 2 PWM
#define PWM2_A          PA8
#define PWM2_B          PA9
#define PWM2_C          PA10
#define PWM2_EN         PA11

// MOTOR 3 PWM
#define PWM3_A          PB6
#define PWM3_B          PB7
#define PWM3_C          PB8
#define PWM3_EN         PB9

// MOTOR 4 PWM
#define PWM4_A          PC6
#define PWM4_B          PC7
#define PWM4_C          PC8
#define PWM4_EN         PC9

// -------------------------- CONFIG

#define ROBOT_ID        0

#define ROBOT_RADIUS    0.072f  // Robot radius in meters
#define WHEEL_RADIUS    0.034f  // Wheel radius in meters
#define MOTOR_PP        11      // Motor Pole Pair Count

#define WHEEL1_ANG      M_PI * (1.0 / 6.0) // 30°
#define WHEEL2_ANG      M_PI * (5.0 / 6.0) // 150°
#define WHEEL3_ANG      M_PI * (5.0 / 4.0) // 225°
#define WHEEL4_ANG      M_PI * (7.0 / 4.0) // 315°

#define SUPPLY_VOLTAGE  18.5
#define SUPPLY_LIMIT    16

#define DEFAULT_KP      0.2f
#define DEFAULT_KI      20.0f
#define DEFAULT_KD      0.0f
#define VEL_RAMP        1000.0f
#define DEFAULT_LPF     0.01f //Low pass filter

#define SLAVE_ADDR      {'R','x','A','A','A'}

#define MAX_TIMES_WITH_NO_MSG 10

// -------------------------- MODE

// Define DEBUG_MODE to enable Serial and motor debugs
// #define DEBUG_MODE

#endif