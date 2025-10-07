#ifndef RADIO_MANAGER_H
#define RADIO_MANAGER_H

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "robot_command.h"

class RadioManager {
private:
    RF24 radio;
    SPIClass radio_spi;
    char dataReceived[11];
    bool newData;
    unsigned int t_without_msg;
    const byte* address;
    uint8_t irq_pin;

public:
    RadioManager(uint8_t ce_pin, uint8_t csn_pin, 
                uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin,
                uint8_t irq_pin, const byte* address);
    
    void init();
    bool checkAndReceive(RobotCommand& cmd);
    bool getAvailable();
};

#endif // RADIO_MANAGER_H