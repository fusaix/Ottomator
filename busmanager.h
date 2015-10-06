/*
* busmamager.h
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/

#ifndef BUSMANAGER_H
#define BUSMANAGER_H

#include <string>
#include "libmodbus/modbus.h"
#include "ottoutils.h"

// Logging as global function
void writeLog(std::string text);

class BusManager
{
public:
    BusManager();
    ~BusManager();

    // Communication setup methods
    void modbusConnectRTU(std::string port);
    void modbusDisConnect();
    bool isConnected();

    // Master communication methods
    int modbusWriteData(int slave, int functionCode, int startAddress, uint16_t data, int verbose = 0);
    std::string modbusReadData(int slave, int functionCode, int startAddress, int noOfItems, int verbose = 0);

    // Data monitoring
    void busDataMonitor(std::string mode, uint8_t* data, uint8_t dataLen);

private:
    modbus_t * m_modbus;
    bool m_connected;
    int m_errors;
    int m_packets;

};

#endif // BUSMANAGER_H
