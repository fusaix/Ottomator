/*
* robot.cpp
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/

#include <iostream>
#include <winsock2.h>
#include <windows.h>
#include <math.h>
#include "robot.h"
#include "busmanager.h"

using namespace std;

Robot::Robot()
{
    positionArray[0] = 1; // x
    positionArray[1] = 1; // y
    positionArray[2] = 1; // z
    positionArray[3] = 1; // p
    positionArray[4] = 2; // v

    for(int i=0; i < 16; ++i)
    {
        deviceDataStatusRegister1[i] = false;
    }
    //cout << "The Robot is created." << endl;
    writeLog("The Robot is created.\n");
}

Robot::~Robot()
{
    //cout << "The Robot is dead." << endl;
    writeLog("The Robot is dead.\n");
}

int Robot::setPosition(int actuator, int position, int timeOut)
{
    uint16_t data;
    data = position-1;
    int max;
    switch(actuator)
    {
        case OttoUtils::xAct:
        case OttoUtils::yAct:
        case OttoUtils::zAct:
            max = 6;
            break;
        case OttoUtils::pAct:
            max = 2;
            break;
        case OttoUtils::vAct:
            int open(-1);
            switch(position)
            {
                case 1:
                    open = true;
                    break;
                case 2:
                    open = false;
                    break;
                default:
                    max = 2;
            }
            if(open != -1) return setCastelPositionTo(open, timeOut); // 0 = time out, 1 = position completed ### TimeOut Château à mesurer.
    }
    if(position > max)
    {
        //cout << "Robot > [Error] setPosition - position unrecognized" << endl;
        writeLog("Robot > [Error] setPosition - position unrecognized\n");
        return 3; // 3 = input error
    }
    int ret[4];
    // Select task
    Sleep(50);
    ret[0] = m_busManager.modbusWriteData(actuator, 0x6, 0x0D03, data); // WriteSingleReg

    // Send start
    Sleep(50);
    ret[1] = m_busManager.modbusWriteData(actuator, 0x5, 0x040C, 0x0000); // WriteSingleCoil
    Sleep(50);
    ret[2] = m_busManager.modbusWriteData(actuator, 0x5, 0x040C, 0xFF00); // WriteSingleCoil
    Sleep(50);
    ret[3] = m_busManager.modbusWriteData(actuator, 0x5, 0x040C, 0x0000); // WriteSingleCoil

    //cout << ret[0] << ", " << ret[1] << ", " << ret[2] << ", " << ret[3] << endl;
    if (ret[0] == 1 && ret[1] == 1 && ret[2] == 1 && ret[3] == 1)
    {
        return completionFor(actuator, position, OttoUtils::position, timeOut); // 0 = time out, 1 = position completed / catch successful (PEND), 2 = push completed without obstacle (PSFL Missed work part)
    } else
    {
        return -1; // -1 = writing error
    }
}

int Robot::setCastelPositionTo(bool open, int timeOut)
{
    int data, ret, actionBitIndex;
    if(open)
    {
        data = 0x1; // binary = 01 (write 0x0001 bit 0)
        actionBitIndex = 6; // (read 0x0000 bit 6)
    } else
    {
        data = 0x2; // binary = 10 (write 0x0001 bit 1)
        actionBitIndex = 7; // (read 0x0000 bit 7)
    }

    Sleep(50);
    ret = m_busManager.modbusWriteData(OttoUtils::vAct, 0x6, 0x0001, data); // WriteSingleReg // Castel intput on address 0x0001
    if(ret !=1) return -1; // -1 = writing error

    int tau(1000);
    int timer(0);
    while(1)
    {
        Sleep(tau);
        updateStatusOfCurrent(OttoUtils::vAct);
        if(!getDeviceDataStatusRegister1Bit(1) ||
                !getDeviceDataStatusRegister1Bit(2) ||
                getDeviceDataStatusRegister1Bit(3) ||
                getDeviceDataStatusRegister1Bit(4) ||
                getDeviceDataStatusRegister1Bit(5) ||
                (!getDeviceDataStatusRegister1Bit(6) && !getDeviceDataStatusRegister1Bit(7))) // Issue capture
        {
            //cout << "Robot > [Error] Issue detected ! " << endl;
            writeLog("Robot > [Error] Issue detected ! \n");
            return 0;
        }
        if(!getDeviceDataStatusRegister1Bit(actionBitIndex)) // !!!!!! read 0x0000 bit 6 for open status, 7 for close status (0 = reached)
        {
            if(open)
            {
                //cout << "Robot > [Info] Castel OPENED." << endl;
                writeLog("Robot > [Info] Castel OPENED.\n");
                setRobotPosition(OttoUtils::vAct, 1);
            } else
            {
                //cout << "Robot > [Info] Castel CLOSED." << endl;
                writeLog("Robot > [Info] Castel CLOSED.\n");
                setRobotPosition(OttoUtils::vAct, 2);
            }
            // Stop motion
            ret = m_busManager.modbusWriteData(OttoUtils::vAct, 0x6, 0x0001, 0x00); // WriteSingleReg // Castel intput on address 0x0001
            if(ret !=1) return -1; // -1 = writing error

            return 1; // 1 = position completed
        } else
        {
            timer += tau;
            if (timer > timeOut)
            {
                //cout << "Robot > [Error] Time out! " << endl;
                writeLog("Robot > [Error] Time out! \n");
                setRobotPosition(OttoUtils::vAct, 0);
                return 0; // 0 = time out
            }
            //cout << "Robot > [Info] " << timer/1000 << "s elapsed..." << endl;
            writeLog("Robot > [Info] " + OttoUtils::numberToString(timer/1000) + "s elapsed...\n");
        }
    }
}


bool Robot::jog(int actuator, int direction, int duration)
{
    uint16_t startAddress;
    switch(direction)
    {
        case -1:
            startAddress = 0x0417;
            break;
        case +1:
            startAddress = 0x0416;
            break;
        default:
        //cout << "Robot > [Error] Jog - direction unrecognized" << endl;
        writeLog("Robot > [Error] Jog - direction unrecognized\n");
        return false;
    }

    // Perfom jog
    m_busManager.modbusWriteData(actuator, 0x5, startAddress, 0xFF00); // WriteSingleCoil
    Sleep(duration); // in milliseconds
    m_busManager.modbusWriteData(actuator, 0x5, startAddress, 0x0000); // WriteSingleCoil

    return true;
}

bool Robot::homing(int actuator)
{
    if(actuator == OttoUtils::yAct)
    {
        //cout << "Robot > [Info] Homing does not apply for this actuator" << endl;
        writeLog("Robot > [Info] Homing does not apply for this actuator\n");
        return false;
    }
    Sleep(50);
    m_busManager.modbusWriteData(actuator, 0x5, 0x040B, 0x0000); // WriteSingleCoil
    Sleep(50);
    m_busManager.modbusWriteData(actuator, 0x5, 0x040B, 0xFF00); // WriteSingleCoil

    return completionFor(actuator, 0, OttoUtils::homing, 60000);
}

void Robot::alarmReset(int actuator)
{
    Sleep(100);
    m_busManager.modbusWriteData(actuator, 0x5, 0x0407, 0xFF00); // WriteSingleCoil
    Sleep(50);
    m_busManager.modbusWriteData(actuator, 0x5, 0x0407, 0x0000); // WriteSingleCoil

    updateStatusOfCurrent(actuator, true);
}


/// Motion information

// Position or Homing Completed ???
int Robot::completionFor(int actuator, int position, int actionBitIndex, int timeOut)
{
    int tau(1000);
    int timer(0);
    while(1)
    {
        Sleep(tau);
        updateStatusOfCurrent(actuator);
        if(getDeviceDataStatusRegister1Bit(15) ||
                !getDeviceDataStatusRegister1Bit(12) ||
                getDeviceDataStatusRegister1Bit(10) ||
                getDeviceDataStatusRegister1Bit(9) ||
                getDeviceDataStatusRegister1Bit(8) ||
                getDeviceDataStatusRegister1Bit(5)) // Issue capture
        {
            //cout << "Robot > [Error] Issue detected ! " << endl;
            writeLog("Robot > [Error] Issue detected ! \n");
            return 0;
        }
        if(getDeviceDataStatusRegister1Bit(11)) // !!!!!! read 0x9005 bit 11 for Missed work part in push-motion operation
        {
            //cout << "Robot > [Info] PSFL = 1." << endl;
            writeLog("Robot > [Info] PSFL = 1.\n");
            if(actuator == OttoUtils::pAct) setRobotPosition(OttoUtils::pAct, 3); // catch successful !!!
            return 2;
        }
        if(getDeviceDataStatusRegister1Bit(actionBitIndex)) // !!!!!! read 0x9005 bit 3 for position, 4 for homing
        {
            if(actionBitIndex == 3)
            {
                //cout << "Robot > [Info] PEND = 1." << endl;
                writeLog("Robot > [Info] PEND = 1.\n");
            } else
            {
                //cout << "Robot > [Info] HEND = 1." << endl;
                writeLog("Robot > [Info] HEND = 1.\n");
            }
            // setRobotPosition !!!
            setRobotPosition(actuator, position);
            return 1;
        } else
        {
            timer += tau;
            if (timer > timeOut)
            {
                //cout << "Robot > [Error] Time out! " << endl;
                writeLog("Robot > [Error] Time out! \n");
                return 0;
            }
            //cout << "Robot > [Info] " << timer/1000 << "s elapsed..." << endl;
            writeLog("Robot > [Info] " + OttoUtils::numberToString(timer/1000) + "s elapsed...\n");
        }
    }
}

void Robot::updateStatusOfCurrent(int actuator, int verbose) // Read and parse data
{
    int wordAdress, trial(5);
    (actuator == OttoUtils::vAct) ? wordAdress = 0x0000 : wordAdress = 0x9005; // Castel output on address 0x0000

    string result;
    do
    {
        result = m_busManager.modbusReadData(actuator, 0x3, wordAdress, 0x0001, 2);
        if(result == "ERROR")
        {
            ++ trial;
        } else
        {
            trial = 0;
        }
    } while(trial > 0);

    //if(verbose > 0) cout << "binary " << result << endl;
    if(verbose > 0) writeLog("binary " + OttoUtils::numberToString(result) + "\n");
    for(unsigned int i=0; i < 16; ++i) // 0..15
    {
        (result[i] == '1') ? deviceDataStatusRegister1[i]=1 : deviceDataStatusRegister1[i]=0;
    }

    if(verbose > 1)
    {
        if(actuator ==  OttoUtils::vAct)
        {
            /*
            cout << "CLOS \t" << deviceDataStatusRegister1[7] << "0 = (Closed or EMGV == 0)" << endl; // 0 = Fermé
            cout << "OPEN \t" << deviceDataStatusRegister1[6] << "0 = (Opened or EMGV == 0)" << endl; // 0 = Ouvert ### défaut capteur : sur incohérence et double 0.
            cout << "MOTO \t" << deviceDataStatusRegister1[5] << endl; // Défaut moteur /// normal = 0
            cout << "P2 \t" << deviceDataStatusRegister1[4] << endl; // Capteur Pince 2 /// normal = 0
            cout << "P1 \t" << deviceDataStatusRegister1[3] << endl; // Capteur Pince 1 /// normal = 0
            cout << "EMGV \t" << deviceDataStatusRegister1[2] << "0 = EMG-" << endl; // EMG- /// normal = 1 (reversed logic compared to IAI actuators)
            cout << "LAT \t" << deviceDataStatusRegister1[1] << "0 = Opened, 1 = Closed" << endl; // Porte latérale ouverte /// normal = 1
            */
            writeLog("CLOS \t" + OttoUtils::numberToString(deviceDataStatusRegister1[7]) + "0 = (Closed or EMGV == 0)\n");
            writeLog("OPEN \t" + OttoUtils::numberToString(deviceDataStatusRegister1[6]) + "0 = (Opened or EMGV == 0))\n");
            writeLog("MOTO \t" + OttoUtils::numberToString(deviceDataStatusRegister1[5]) + "\n");
            writeLog("P2 \t" + OttoUtils::numberToString(deviceDataStatusRegister1[4]) + "\n");
            writeLog("P1 \t" + OttoUtils::numberToString(deviceDataStatusRegister1[3]) + "\n");
            writeLog("EMGV \t" + OttoUtils::numberToString(deviceDataStatusRegister1[2]) + "0 = EMG-\n");
            writeLog("LAT \t" + OttoUtils::numberToString(deviceDataStatusRegister1[1]) + "0 = Opened, 1 = Closed\n");
        } else
        {
            /*
            cout << "EMGS \t" << deviceDataStatusRegister1[15] << endl; // normal = 0
            cout << "SFTY \t" << deviceDataStatusRegister1[14] << endl;
            cout << "PWR \t" << deviceDataStatusRegister1[13] << endl;
            cout << "SV \t" << deviceDataStatusRegister1[12] << endl; // Servo ON status
            cout << "PSFL \t" << deviceDataStatusRegister1[11] << endl; // Missed work part in push-motion operation
            cout << "ALMH \t" << deviceDataStatusRegister1[10] << endl;
            cout << "ALML \t" << deviceDataStatusRegister1[9] << endl; // Minor failure status
            cout << "ABER \t" << deviceDataStatusRegister1[8] << endl;
            cout << "BKRL \t" << deviceDataStatusRegister1[7] << endl;
            cout << "STP \t" << deviceDataStatusRegister1[5] << endl;
            cout << "HEND \t" << deviceDataStatusRegister1[4] << endl; // Home return completion status
            cout << "PEND \t" << deviceDataStatusRegister1[3] << endl; // Position complete status
            cout << "CEND \t" << deviceDataStatusRegister1[2] << endl;
            cout << "CLBS \t" << deviceDataStatusRegister1[1] << endl;
            */
            writeLog("EMGS \t" + OttoUtils::numberToString(deviceDataStatusRegister1[15]) + "\n");
            writeLog("SFTY \t" + OttoUtils::numberToString(deviceDataStatusRegister1[14]) + "\n");
            writeLog("PWR \t" + OttoUtils::numberToString(deviceDataStatusRegister1[13]) + "\n");
            writeLog("SV \t" + OttoUtils::numberToString(deviceDataStatusRegister1[12]) + "\n");
            writeLog("PSFL \t" + OttoUtils::numberToString(deviceDataStatusRegister1[11]) + "\n");
            writeLog("ALML \t" + OttoUtils::numberToString(deviceDataStatusRegister1[10]) + "\n");
            writeLog("ALMH \t" + OttoUtils::numberToString(deviceDataStatusRegister1[9]) + "\n");
            writeLog("ABER \t" + OttoUtils::numberToString(deviceDataStatusRegister1[8]) + "\n");
            writeLog("BKRL \t" + OttoUtils::numberToString(deviceDataStatusRegister1[7]) + "\n");
            writeLog("STP \t" + OttoUtils::numberToString(deviceDataStatusRegister1[5]) + "\n");
            writeLog("HEND \t" + OttoUtils::numberToString(deviceDataStatusRegister1[4]) + "\n");
            writeLog("PEND \t" + OttoUtils::numberToString(deviceDataStatusRegister1[3]) + "\n");
            writeLog("CEND \t" + OttoUtils::numberToString(deviceDataStatusRegister1[2]) + "\n");
            writeLog("CLBS \t" + OttoUtils::numberToString(deviceDataStatusRegister1[1]) + "\n");

        }
    }

}

void Robot::setRobotPosition(int actuator, int value)
{
    positionArray[actuator-1] = value;
}

int Robot::getRobotPosition(int actuator)
{
    return positionArray[actuator-1];
}


/// Hardware management
/// -------------------

bool Robot::activateMODBUS(int actuator)
{
    Sleep(100);
    int writeSuccessful = m_busManager.modbusWriteData(actuator, 0x5, 0x0427, 0xFF00); // WriteSingleCoil
    updateStatusOfCurrent(actuator);
    return writeSuccessful;
}

bool Robot::servo(bool onOff, int actuator)
{
    Sleep(100);
    (onOff) ? m_busManager.modbusWriteData(actuator, 0x5, 0x0403, 0xFF00) : m_busManager.modbusWriteData(actuator, 0x5, 0x0403, 0x0000); // WriteSingleCoil
    updateStatusOfCurrent(actuator);
    return deviceDataStatusRegister1[12];
}

bool Robot::getDeviceDataStatusRegister1Bit(int index)
{
    return deviceDataStatusRegister1[index];
}


/// Task matrix definition getters

//int Robot::xOfSample(int n)
//{
//  int x=n%5;
//  if(x == 0) x = 5;
//  if(n == POSITION_REST    ) x = 4;
//  if(n == POSITION_DETECTOR) x = 6;
//  if(n == POSITION_ZERO    ) x = 1;
//  return x;
//}
//
//int Robot::yOfSample(int n)
//{
//  //int y= ceil(n/5.0)+1;
//  int y=(n+4+5)/5;
//  return y;
//}




