#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

#include <Arduino.h>
#include "sbus.h"
#include "Config.h"

class SbusReceiver {
public:
    void begin();
    void update();

    float getRoll();     // CH1
    float getPitch();    // CH2
    float getThrottle(); // CH3
    float getYaw();      // CH4
    bool isSignalLost();
    float getRawChannel(int ch);    // optional öffentlich
    bool isArmedSwitchOn();         // neu
    



private:
    bfs::SbusRx m_sbus = bfs::SbusRx(&Serial1, SBUS_RX_PIN, -1, SBUS_INVERTED);
    bfs::SbusData m_data;

    float mapChannel(int raw, bool symmetric = true);

    unsigned long lastSignalTime = 0;
    bool signalAvailable = false;

    // Geglättete Kanalwerte (Low-Pass Filter)
    float m_filteredRoll = 0.0f;
    float m_filteredPitch = 0.0f;
    float m_filteredThrottle = 0.0f;
    float m_filteredYaw = 0.0f;

};

#endif
