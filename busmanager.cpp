/*
* busmanager.cpp
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/

#include "busmanager.h"
#include <iostream>
#include <errno.h>
#include <iomanip>
#include <sstream>

using namespace std;

BusManager* m_instance;

BusManager::BusManager() :
    m_modbus(NULL),
    m_connected(false),
    m_errors(0),
    m_packets(0)
{
    //cout << "The BusManager is created." << endl;
    writeLog("The BusManager is created.\n");
}

BusManager::~BusManager()
{
    //cout << "The BusManager is dead." << endl;
    writeLog("The BusManager is dead.\n");
}

void BusManager::modbusConnectRTU(std::string port)
{
    //Modbus RTU connect
    string line;
    modbusDisConnect();

    //cout << "BusManager > [Info] " <<  "Modbus Connect RTU" << endl;
    writeLog("BusManager > [Info] Modbus Connect RTU\n");

    m_modbus = modbus_new_rtu(port.c_str(), 115200, 'N', 8, 1, MODBUS_RTU_RTS_NONE);
    line = "Connecting to Serial Port [" + port + "]...";
    //cout << "BusManager > [Info] " <<  line << endl;
    writeLog("BusManager > [Info] " + line +"\n");

    //Debug messages from libmodbus
    #ifdef LIB_MODBUS_DEBUG_OUTPUT
        modbus_set_debug(m_modbus, 1);
    #endif

    if(m_modbus && modbus_connect(m_modbus) == -1) {
        //cout << "BusManager > [Error] " <<  "Connection failed. Could not connect to serial port" << endl;
        writeLog("BusManager > [Error] Connection failed. Could not connect to serial port\n");
        m_connected = false;
        line += "Failed";
    }
    else {
        m_connected = true;
        line += "OK";
        //cout << "BusManager > [Info] " <<  line << endl;
        writeLog("BusManager > [Info] " +  line + "\n");
    }

    //Add timeline
    //cout << OttoUtils::SysTimeStamp() << " : " << line << endl;
    writeLog(OttoUtils::SysTimeStamp() + " : " + line + "\n");

}


void BusManager::modbusDisConnect()
{
    //Modbus disconnect
    //cout <<  "Modbus disconnected" << endl;
    writeLog("Modbus disconnected\n");

    if(m_modbus) {
        modbus_close(m_modbus);
        modbus_free(m_modbus);
        m_modbus = NULL;
    }

    m_connected = false;
}

bool BusManager::isConnected()
{
    //Modbus is connected
    return m_connected;
}


int BusManager::modbusWriteData(int slave, int functionCode, int startAddress, uint16_t data, int verbose)
{
    //if(verbose > 1) cout << "BusManager > [Info] " << "Modbus Write Data " << endl;
    if(verbose > 1) writeLog("BusManager > [Info] Modbus Write Data \n");

    if(m_modbus == NULL) return 0;

    int ret = -1; //return value from functions

    modbus_set_slave(m_modbus, slave);
    //request data from modbus
    switch(functionCode)
    {
        case _FC_WRITE_SINGLE_COIL:
            ret = modbus_write_bit(m_modbus, startAddress, data);
            //noOfItems = 1;
            break;

        case _FC_WRITE_SINGLE_REGISTER:
            ret = modbus_write_register(m_modbus, startAddress, data);
            //noOfItems = 1;
            break;

        /*
         * case _FC_WRITE_MULTIPLE_COILS:
        {
                uint8_t * dat = new uint8_t[noOfItems];
                for(int i = 0; i < noOfItems; ++i)
                {
                        dat[i] = data[i];
                }
                ret = modbus_write_bits(m_modbus, startAddress, noOfItems, dat);
                //delete[] data;
                break;
        }
        case _FC_WRITE_MULTIPLE_REGISTERS:
        {
                uint16_t * dat = new uint16_t[noOfItems];
                for(int i = 0; i < noOfItems; ++i)
                {
                        dat[i] = data[i];
                }
                ret = modbus_write_registers(m_modbus, startAddress, noOfItems, dat);
                //delete[] data;
                break;
        }*/

        default:
            break;
    }

    //if(verbose > 1) cout << "BusManager > [Info] " <<  "Modbus Write Data return value = " << ret << endl;
    if(verbose > 1) writeLog("BusManager > [Info] Modbus Write Data return value = " + OttoUtils::numberToString(ret) + "\n");

    //update data model
    if(ret == 1) //noOfItems = 1
    {
        //values written correctly
        //if(verbose > 1) cout << OttoUtils::SysTimeStamp() << " : values written correctly." << endl;
        if(verbose > 1) writeLog(OttoUtils::SysTimeStamp() + " : values written correctly." + "\n");
        return ret;
    } else
    {
        ++m_errors;

        string line;
        if(ret < 0)
        {
            /*
            char buffer [3];
            char *intStr = itoa(ret,buffer,10);
            string str = string(intStr);*/
            line = string("Slave threw exception  >  ") + OttoUtils::numberToString(ret) + " " +  modbus_strerror(errno);
        }
        else {
            line = string("Number of registers returned does not match number of registers requested!. [")  +  modbus_strerror(errno) + "]";
        }
        //if(verbose > 0) cout << "BusManager > [Error] " <<  "Modbus Write Data failed. " << endl;
        //if(verbose > 0) cout << OttoUtils::SysTimeStamp() << " : " << line << endl;
        if(verbose > 0) writeLog("BusManager > [Error] Modbus Write Data failed. \n");
        if(verbose > 0) writeLog(OttoUtils::SysTimeStamp() + " : " + line + "\n");

        modbus_flush(m_modbus); //flush data
        return ret;
    }

}


string BusManager::modbusReadData(int slave, int functionCode, int startAddress, int noOfItems, int verbose)
{
    //if(verbose > 1) cout << "BusManager > [Info] " <<  "Modbus Read Data " << endl;
    if(verbose > 1) writeLog("BusManager > [Info] Modbus Read Data \n");

    if(m_modbus == NULL) return 0;

    uint8_t dest[1024]; //setup memory for data
    uint16_t * dest16 = (uint16_t *) dest;
    memset(dest, 0, 1024);
    int ret = -1; //return value from read functions
    bool is16Bit = false;

    modbus_set_slave(m_modbus, slave);
    //request data from modbus
    switch(functionCode)
    {
        case _FC_READ_COILS:
            ret = modbus_read_bits(m_modbus, startAddress, noOfItems, dest);
            break;
        case _FC_READ_DISCRETE_INPUTS:
            ret = modbus_read_input_bits(m_modbus, startAddress, noOfItems, dest);
            break;
        case _FC_READ_HOLDING_REGISTERS:
            ret = modbus_read_registers(m_modbus, startAddress, noOfItems, dest16);
            is16Bit = true;
            break;
        case _FC_READ_INPUT_REGISTERS:
            ret = modbus_read_input_registers(m_modbus, startAddress, noOfItems, dest16);
            is16Bit = true;
            break;
        default:
            break;
    }

    //if(verbose > 1) cout << "BusManager > [Info] " <<  "Modbus Read Data return value = " << ret << endl;
    if(verbose > 1) writeLog("BusManager > [Info] Modbus Read Data return value = " + OttoUtils::numberToString(ret) + "\n");

    //update data model
    string result;
    string binResult;
    if(ret == noOfItems)
    {
        for(int i = 0; i < noOfItems; ++i)
        {
            int data = is16Bit ? dest16[i] : dest[i];
            result += (OttoUtils::numberToString(data) + " ");
            binResult += (OttoUtils::decToBin(data, true) + " ");
        }        
        // Display result
        //if(verbose > 1) cout << OttoUtils::RxTimeStamp() <<  " : int data = " << result << endl;
        if(verbose > 1) writeLog(OttoUtils::RxTimeStamp() +  " : int data = " + result + "\n");

        // Return binResult
        return binResult;
    } else
    {
        m_errors += 1;

        string line;
        if(ret < 0)
        {
            line = string("Slave threw exception  >  ") + OttoUtils::numberToString(ret) + " " +  modbus_strerror(errno);
        } else
        {
            line = string("Number of registers returned does not match number of registers requested!. [")  +  modbus_strerror(errno) + "]";
        }
        //if(verbose > 0) cout << "BusManager > [Error] " <<  "Modbus Read Data failed. "  << endl;
        //if(verbose > 0) cout << OttoUtils::SysTimeStamp() << " : " << line << endl;
        if(verbose > 0) writeLog("BusManager > [Error] Modbus Read Data failed. \n");
        if(verbose > 0) writeLog(OttoUtils::SysTimeStamp() + " : " + line + "\n");

        modbus_flush(m_modbus); //flush data
        return "ERROR";
    }
}

void BusManager::busDataMonitor(string mode, uint8_t* data, uint8_t dataLen)
{
    //Request Raw data from port - Update raw data model
    string line;
    stringstream ss;

    for(int i = 0; i < dataLen; ++i)
    {   // Construct line
        ss << uppercase << " " << hex << setw(2) << setfill('0') << (int) (unsigned char) data[i];
    }
    line = ss.str();

    if(mode == "TX")
    {
        //cout << OttoUtils::TxTimeStamp() << " :"  << line  << endl;
        writeLog(OttoUtils::TxTimeStamp() + " : " + line + "\n");
    } else if(mode == "RX")
    {
        //cout << OttoUtils::RxTimeStamp() << " :"  << line  << endl;
        writeLog(OttoUtils::RxTimeStamp() + " : " + line + "\n");
    } else
    {
        //cout << "BusManager > [Error] : busDataMonitor mode unrecognized" << endl;
        writeLog("BusManager > [Error] : busDataMonitor mode unrecognized\n");
    }
}

extern "C" {
    void busMonitorRawResponseData(uint8_t * data, uint8_t dataLen)
    {
        m_instance->busDataMonitor("RX", data, dataLen);
    }

    void busMonitorRawRequestData(uint8_t * data, uint8_t dataLen)
    {
        m_instance->busDataMonitor("TX", data, dataLen);
    }
}
