/*
* ottomator.h
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/

#ifndef OTTOMATOR_H
#define OTTOMATOR_H

#include <string>
#include <vector>
#include "robot.h"
#include "ottoutils.h"

#define Stuck_in_birth_sequence         -1
#define Stuck_in_initiation_sequence    -2
#define Stuck_in_fetch_sequence         -3
#define Stuck_in_putback_sequence       -4
#define Stuck_in_finish_sequence        -5
#define Sequence_case_out_of_range      -6
#define Empty_catch                     -7
#define Birth_completed                 -8
#define Initiation_completed            -9
#define Fetch_completed                 -10
#define Putback_completed               -11
#define Finish_completed                -12
#define Need_input                      -13

#define SV      1
#define STP     2
#define EMGS    4
#define EMGV    8
#define P1      16
#define P2      32
#define LAT     64
#define ALML    128
#define ALMH    256
#define ABER    512
#define MOTO    1024
#define XCAS    2048


class Ottomator
{
public:
    Ottomator();
    ~Ottomator();

    // Utilities
    std::string getAllStatusesOfDSS1ForBit(int index, bool verbose = false);
    void updateM_StatusMatrix();
    void resetM_StatusMatrix();
    int manageStatusGate();
    int manageNextFor(int sequence, int actionSuccess, int error, int specialIndex = 0, int specialManagement = 0);
    bool allServo(bool onOff);

    // Main Sequences
    int birthSequence();
    int frameworkSequenceTo(bool initiate);
    int workSequenceTo(bool fetch, int sampleN);

    // Messages
    virtual   const char*    GetErrorTextFrench(int i);
    //virtual   const char*    GetErrorTextEnglish(int i); // D'abord le francais, mais apres il faudra traduire

    // Demontrators
    int cycler();
    void displayM_StatusMatrix();

    // Case getters and setters
    int** getM_StatusMatrix();
    int getBirthSequenceCase();
    void setBirthSequenceCase(int n);
    int getFrameworkSequenceCase();
    void setFrameworkSequenceCase(int n);
    int getWorkSequenceCase();
    void setWorkSequenceCase(int n);
    std::vector<std::string> getM_sampleArray();
    void setM_sampleArray(std::vector<std::string> sampleArray);
    int getNCurrentSample();
    void setNCurrentSample(int n);

    // Components
    Robot m_robot;

    // Task matrix definition getters
    virtual void  xyOfSample(int n, int& x, int& y);
    inline  int   xOfSample(int n) { int x,y; xyOfSample(n, x, y); return x; }
    inline  int   yOfSample(int n) { int x,y; xyOfSample(n, x, y); return y; }

private:
    // State attributs
    int m_statusMatrix[16][6];

    // Cycler attibuts
    std::vector<std::string> m_sampleArray;

};

#endif // OTTOMATOR_H
