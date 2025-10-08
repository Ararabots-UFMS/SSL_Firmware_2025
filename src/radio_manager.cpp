#include "configs.h"
#include "radio_manager.h"
#include <SimpleFOC.h>

RadioManager::RadioManager(uint8_t ce_pin, uint8_t csn_pin, 
                          uint8_t mosi_pin, uint8_t miso_pin, uint8_t sck_pin,
                          uint8_t irq_pin, const byte* address) :
    radio(ce_pin, csn_pin),
    radio_spi(mosi_pin, miso_pin, sck_pin),
    newData(false),
    t_without_msg(0),
    irq_pin(irq_pin),
    address(address)
{
    // Constructor initializes variables
}

void RadioManager::init() {
    pinMode(irq_pin, INPUT);
    
    radio_spi.begin();
    radio.begin(&radio_spi);
    radio.setDataRate(RF24_2MBPS);
    radio.openReadingPipe(1, address);
    radio.setChannel(120);
    radio.startListening();
}

bool RadioManager::checkAndReceive(RobotCommand& cmd) {
    if (radio.available()) {
        radio.read(&dataReceived, sizeof(dataReceived));
        newData = true;
        
        // Decode the received data
        cmd = decodeCommand(dataReceived);
        noSignalMonitor = 0;
        return true;
    }
    else
        noSignalMonitor++;
    return false;
}

bool RadioManager::getSignal(float * wheels)
{
    if(noSignalMonitor >= MAX_TIMES_WITH_NO_MSG)
    {
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = 0;
        }
        return false;
    }
    else
    {
        return true;
    }
}