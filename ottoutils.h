/*
* ottoutils.h
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/


#ifndef OTTOUTILS_H
#define OTTOUTILS_H

#include <string>
#include <time.h>
#include <sstream>
#include <math.h>
#include <vector>
#include "libmodbus/modbus-private.h"

static const std::string ModbusFunctionNames[]={"Read Coils (0x01)","Read Discrete Inputs (0x02)","Read Holding Registers (0x03)",
                               "Read Input Registers (0x04)","Write Single Coil (0x05)","Write Single Register (0x06)",
                               "Write Multiple Coils (0x0f)","Write Multiple Registers (0x10)"};
static const std::string symbols[]={"", "CLBS", "CEND", "PEND", "HEND", "STP", "", "BKRL", "ABER", "ALML", "ALMH", "PSFL", "SV", "PWR", "SFTY", "EMGS"};

static const int ModbusFunctionCodes[]={0x1,0x2,0x3,0x4,0x5,0x6,0xf,0x10};


class OttoUtils
{
public:
    OttoUtils();

    static std::string ModbusDataTypeName(int fCode)
    {
        switch(fCode)
        {
            case _FC_READ_COILS:
            case _FC_WRITE_SINGLE_COIL:
            case _FC_WRITE_MULTIPLE_COILS:
                return "Coil (binary)";
            case _FC_READ_DISCRETE_INPUTS:
                return "Discrete Input (binary)";
            case _FC_READ_HOLDING_REGISTERS:
            case _FC_WRITE_SINGLE_REGISTER:
            case _FC_WRITE_MULTIPLE_REGISTERS:
                return "Holding Register (16 bit)";
            case _FC_READ_INPUT_REGISTERS:
                return "Input Register (16 bit)";
            default:
                break;
        }
        return "Unknown";
    }

    static bool ModbusIsWriteFunction(int fCode)
    {
        switch(fCode)
        {
            case _FC_READ_COILS:
            case _FC_READ_DISCRETE_INPUTS:
            case _FC_READ_HOLDING_REGISTERS:
            case _FC_READ_INPUT_REGISTERS:
                return false;

            case _FC_WRITE_SINGLE_COIL:
            case _FC_WRITE_MULTIPLE_COILS:
            case _FC_WRITE_SINGLE_REGISTER:
            case _FC_WRITE_MULTIPLE_REGISTERS:
                return true;

            default:
                break;
        }
        return false;
    }

    static bool ModbusIsWriteCoilsFunction(int fCode)
    {
        switch(fCode)
        {
            case _FC_READ_COILS:
            case _FC_READ_DISCRETE_INPUTS:
            case _FC_READ_HOLDING_REGISTERS:
            case _FC_READ_INPUT_REGISTERS:
            case _FC_WRITE_SINGLE_REGISTER:
            case _FC_WRITE_MULTIPLE_REGISTERS:
                return false;

            case _FC_WRITE_SINGLE_COIL:
            case _FC_WRITE_MULTIPLE_COILS:
                return true;

            default:
                break;
        }
        return false;
    }

    static bool ModbusIsWriteRegistersFunction(int fCode)
    {
        switch(fCode)
        {
            case _FC_READ_COILS:
            case _FC_READ_DISCRETE_INPUTS:
            case _FC_READ_HOLDING_REGISTERS:
            case _FC_READ_INPUT_REGISTERS:
            case _FC_WRITE_SINGLE_COIL:
            case _FC_WRITE_MULTIPLE_COILS:
                return false;

            case _FC_WRITE_SINGLE_REGISTER:
            case _FC_WRITE_MULTIPLE_REGISTERS:
                return true;

            default:
                break;
        }
        return false;
    }

    static std::string ModbusFunctionName(int index)
    {
        return ModbusFunctionNames[index];
    }

    static int ModbusFunctionCode(int index)
    {
        return ModbusFunctionCodes[index];
    }

    static std::string TimeStamp()
    {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
        std::string str(buffer);
        return (str);
    }

    static std::string TxTimeStamp()
    {
        return ("Tx > " + TimeStamp());
    }

    static std::string RxTimeStamp()
    {
        return ("Rx > " + TimeStamp());
    }

    static std::string SysTimeStamp()
    {
        return ("Sys > " + TimeStamp());
    }

    template <typename T> static std::string numberToString (T Number)
    {
        std::ostringstream  ss;
        ss << Number;
        return ss.str();
    }

    static std::string decToBin(int number, bool littleEndianFirst = false)
    {
        std::string result = "";
        do
        {
            ((number & 1) == 0) ? result += "0" : result += "1";
            number >>= 1;
        } while(number);

        if(!littleEndianFirst)
        {
            result = OttoUtils::reverse(result);
        }

        // byte completion
        unsigned int completeByteLength = (ceil(result.length()/8.0))*8;
        while(result.length() < completeByteLength)
        {
            littleEndianFirst ? result += "0" : result = "0" + result;
        }

        return result;
    }

    static std::string reverse(std::string to_reverse){
        std::string result;
        for (int i = to_reverse.length()-1; i >=0 ; i--)
            result += to_reverse[i];
        return result;
    }

    static char parity(std::string p)
    {
        //the first char is what we need
        return p.at(0);
    }

    static enum {RTU = 0, TCP = 1, None = 0} ModbusMode;

    static enum {Bin = 2, UInt = 10, SInt = 11, Hex = 16} NumberFormat;

    static enum {ReadCoils = 0x1, ReadDisInputs = 0x2,
                ReadHoldRegs = 0x3, ReadInputRegs = 0x4,
                WriteSingleCoil = 0x5, WriteSingleReg = 0x6,
                WriteMultiCoils = 0xf, WriteMultiRegs = 0x10} FunctionCodes;

    static enum {xAct = 0x1, yAct = 0x2, zAct = 0x3, pAct = 0x4, vAct = 0x5} actuator;

    static enum {off = 0, on = 1, direction = 2, position = 3, homing = 4,
                 fetchPutback = 5, sampleNb = 6, extendedSampleNb = 7, sampleArray = 8} action;

    static std::string formatValue(int value, int frmt, bool is16Bit);

};

#endif // OTTOUTILS_H
