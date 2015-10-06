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

class Ottomator
{
public:
    Ottomator();
    ~Ottomator();

    // Utilities
    std::string getAllStatusesOfDSS1ForBit(int index, bool verbose = false);
    void updateM_StatusMatrix();
    void resetM_StatusMatrix();
    std::vector<std::string> manageStatusGate();
    int manageNextFor(int sequence, int actionSuccess, int error, int specialIndex = 0, int specialManagement = 0);

    // Main Sequences
    std::vector<std::string> birthSequence();
    std::vector<std::string> frameworkSequenceTo(bool initiate);
    std::vector<std::string> workSequenceTo(bool fetch, int sampleN);

    // Messages
    virtual   const char*    GetErrorTextFrench(int i);
    //virtual   const char*    GetErrorTextEnglish(int i); // D'abord le français, mais après il faudra traduire

    // Demontrators
    std::vector<std::string> cycler();
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
    virtual   void           xyOfSample(int n, int& x, int& y);
    inline    int            xOfSample(int n) { int x,y; xyOfSample(n, x, y); return x; }
    inline    int            yOfSample(int n) { int x,y; xyOfSample(n, x, y); return y; }

private:
    // State attributs
    int m_statusMatrix[16][6];

    // Cycler attibuts
    std::vector<std::string> m_sampleArray;

};

#endif // OTTOMATOR_H
