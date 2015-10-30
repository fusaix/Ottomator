/*
* main.cpp
* Ottomator100
*
* Created by Louis CHEN on 03/08/15
* Copyright (c) 2015 ITECH INSTRUMENTS. All rights reserved.
*/

#include <iostream>
#include <stdlib.h>
#include <winsock2.h>
#include <windows.h>
#include "ottomator.h"
#include "robot.h"
#include "busmanager.h"
#include "ottoutils.h"

using namespace std;

int CurrentPosition(0);
int isSampleIn(0);

void writeLog(string text)
{
    cout << text;
}

void connectToMobus(Ottomator& theSequencer)
{
    cout << "Please enter port name: ";
    string port;
    cin >> port;
    theSequencer.m_robot.m_busManager.modbusConnectRTU(port); // Eg: "COM9"
}

int inputActuator()
{
    int actuator(0);
    char a;
    bool redo;
    do
    {
        cout << "Actuator? (x, y, z, p, v) ";
        cin >> a;
        a = toupper(a);
        redo = false;
        switch(a)
        {
            case 'X':
                actuator = 1;
                break;
            case 'Y':
                actuator = 2;
                break;
            case 'Z':
                actuator = 3;
                break;
            case 'P':
                actuator = 4;
                break;
            case 'V':
                actuator = 5;
                break;
            default:
                redo = true;
         }
    } while(redo);

    return actuator;
}

int input(int action)
{
    string n("");
    string nbArray[23];
    for(int i=0; i < 23; ++i)
    {
        nbArray[i] = OttoUtils::numberToString(i);
    }
    bool redo;
    int j;
    do
    {
        j = -1;
        switch(action)
        {
            case OttoUtils::on:
                cout << "On/Off? (1, 0) ";
                break;
            case OttoUtils::position:
                cout << "Position? (1..6) ";
                break;
            case OttoUtils::direction:
                cout << "Direction? (-1, +1) ";
                break;
            case OttoUtils::fetchPutback:
                cout << "Fetch (1) or Putback (0)? ";
                break;
            case OttoUtils::sampleNb:
                cout << "Which sample? (1..20) ";
                break;
            case OttoUtils::extendedSampleNb:
                cout << "Which sample? (0..22) ";
        }
        cin >> n;
        for(int i=0; i < 23; ++i)
        {
            if(n == nbArray[i]) j = i;
        }
        (((j > 0 && j < 7) && action == OttoUtils::position) ||
                ((n == "-1" || n == "1") && action == OttoUtils::direction) ||
                ((n == "0" || n == "1") && (action == OttoUtils::on || action == OttoUtils::fetchPutback)) ||
                ((j > 0 && j < 21) && action == OttoUtils::sampleNb) ||
                ((j > -1 && j < 23) && action == OttoUtils::extendedSampleNb)
                ) ? redo = false : redo = true;
    } while(redo);

    return atoi(n.c_str());
}

void elementaryMenu(Ottomator& theSequencer)
{
    char command(0);
    int actuator(0), n(0);

    do
    {
        // Display list of commands
        cout << "***********************************************" << endl;
        cout << "A - Please select an elementary command to run:" << endl;
        cout << "***********************************************" << endl;
        cout << "1 - setPosition(int actuator, int position) " << endl;
        cout << "2 - jog(int actuator, int direction) " << endl;
        cout << "3 - homing(int actuator) " << endl;
        cout << "4 - activateMODBUS(int actuator) " << endl;
        cout << "5 - servo(bool onOff, int actuator) " << endl;
        cout << "6 - alarmReset(int actuator) " << endl;
        cout << "7 - updateStatusOfCurrent(actuator) " << endl;
        cout << "8 - modbusConnectRTU(port) " << endl;
        cout << "9 - 999999999999999" << endl;
        cout << "0 - back" << endl;
        cout << "Your choice is: " ;
        cin >> command;

        switch(command)
        {
            // 1 - setPosition(int actuator, int position)
            case '1':
                actuator = inputActuator();
                n = input(OttoUtils::position);
                // Perform
                theSequencer.m_robot.setPosition(actuator, n);
                break;
            // 2 - jog(int actuator, int direction)
            case '2':
                actuator = inputActuator();
                n = input(OttoUtils::direction);
                // Perform
                theSequencer.m_robot.jog(actuator, n);
                break;
            // 3 - homing(int actuator)
            case '3':
                // Perform
                actuator = inputActuator();
                theSequencer.m_robot.homing(actuator);
                break;
            // 4 - activateMODBUS(int actuator)
            case '4':
                // Perform
                actuator = inputActuator();
                theSequencer.m_robot.activateMODBUS(actuator);
                break;
            // 5 - servo(bool onOff, int actuator)
            case '5':
                // Perform
                n = input(OttoUtils::on);
                actuator = inputActuator();
                theSequencer.m_robot.servo(n, actuator);
                break;
            // 6 - alarmReset(int actuator)
            case '6':
                // Perform
                actuator = inputActuator();
                theSequencer.m_robot.alarmReset(actuator);
                break;
            // 7 - updateStatusOfCurrent(actuator)
            case '7':
                actuator = inputActuator();
                theSequencer.m_robot.updateStatusOfCurrent(actuator, 2);
                break;
            // 8 - modbusConnectRTU(port)
            case '8':
                connectToMobus(theSequencer);
                break;
            // 9 -
            case '9':
                theSequencer.m_robot.m_busManager.modbusReadData(OttoUtils::xAct, 0x03, 0x9000, 2, 10, 2);
                break;
            case '0':
                command = 0;
                break;
            default:
                cout << "Main > [Error] Invalid command" << endl;
        }
    }
    while (command != 0);

}

void basicPickSample(Robot& R)
{
    // Lift up
    cout << "********** Lift up **********" << endl;
    R.setPosition(3, 1);
    // Open
    cout << "********** Open **********" << endl;
    R.setPosition(4, 1);
    // Drop down
    cout << "********** Drop down **********" << endl;
    R.setPosition(3, 2);
    // Close
    cout << "********** Close **********" << endl;
    R.setPosition(4, 2);
    Sleep(2000);
    // Lift up
    cout << "********** Lift up **********" << endl;
    R.setPosition(3, 1);
}

void basicPutSample(Robot& R, int destination)
{
    cout << "********** Drop down **********" << endl;
    R.setPosition(3, destination);
    cout << "********** Open **********" << endl;
    R.setPosition(4, 1);
    cout << "********** Lift up **********" << endl;
    R.setPosition(3, 1);
}

void goToCastel(Ottomator& theSequencer)
{
    cout << "********** Go to Castel : Y **********" << endl;
    theSequencer.m_robot.setPosition(2, theSequencer.yOfSample(POSITION_DETECTOR));
    cout << "********** Go to Castel : X **********" << endl;
    theSequencer.m_robot.setPosition(1, theSequencer.xOfSample(POSITION_DETECTOR)); // long motion
}

void goToSample(int sampleN, Ottomator& theSequencer)
{
    cout << "********** Go to Sample N : X **********" << endl;
    theSequencer.m_robot.setPosition(1, theSequencer.xOfSample(sampleN));
    cout << "********** Go to Sample N : Y **********" << endl;
    theSequencer.m_robot.setPosition(2, theSequencer.yOfSample(sampleN));
}

void inputSampleArray(Ottomator& theSequencer)
{
    vector<string> sampleArray;
    string input;
    cout << "Enter sample array: (eg: 1.2.3.5.19)" << endl;
    cout << "Array = ";
    cin >> input;

    stringstream ss;
    ss << input;
    int found;
    string temp;
    while(getline(ss, temp, '.'))
    {
        if(stringstream(temp) >> found)
        {   // write buffer
            sampleArray.push_back(OttoUtils::numberToString(found));
        }
    }
    theSequencer.setM_sampleArray(sampleArray);
    int nC = theSequencer.getNCurrentSample();
    theSequencer.resetM_StatusMatrix();
    theSequencer.setNCurrentSample(nC);
}

bool overWriteM_sampleArray(vector<string> &messages)
{
    bool overWrite(false);
    cout << "The current array is: ";
    for (unsigned int i = 0; i < messages.size(); ++i)
        cout << messages[i] << '.';
    cout << endl;
    cout << "Do you want to overwrite this array? Yes? (1) / No? (0) ";
    cin >> overWrite;
    return overWrite;
}

bool measuringFinished()
{
    string ans;
    cout << "You can know measure..." << endl;
    cout << "Measuring finished? (1) / Abort? (0) ";
    cin >> ans;
    return (ans != "0");
}


/////////////////////////////////////////////////////////////////////////////
/// Code in InterWinner



int Language()
{
    return 2; // French
}

void ReportError(int number, const char* text){
    cout << "----------ERROR----------" << endl;
    cout << number << endl;
    cout << text << endl;
    cout << "-------------------------" << endl;

}

void AbortSequence(int number)
{
    cout << "----------SEQUENCE ABORTED----------" << endl;
    cout << number << endl;
    cout << "------------------------------------" << endl;
}

void BgStartAcq(Ottomator& theSequencer) /// theSequencer à enlever
{
    bool fatal(false), problemSolved(false);
    int trial(0);
    string s;
    while (1) {
        DWORD ErrorCode = theSequencer.InsertSample(CurrentPosition); /// theSequencer à enlever
        if (!ErrorCode) { // everything is ok, go for it
            isSampleIn = CurrentPosition;
            return;
        }

        // Display
        s = "Cyclope2: " + theSequencer.GetErrorText(ErrorCode, Language());
        ReportError(3, s.c_str());

        // Manage error

        switch(ErrorCode)
        {
            case CYCLOPE2_SERVO_OFF:
                problemSolved = theSequencer.solveCYCLOPE2_SERVO_OFF();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_STOP_BIT:
                problemSolved = theSequencer.solveCYCLOPE2_STOP_BIT();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_ALARME_ACQUITTABLE:
                problemSolved = theSequencer.solveCYCLOPE2_ALARME_ACQUITTABLE();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_COLLISION_PINCE:
                theSequencer.solveCYCLOPE2_COLLISION_PINCE();
            case CYCLOPE2_COLLISION_CHATEAU:
            case CYCLOPE2_DANGER_TIME_OUT:
            case CYCLOPE2_ALARME_GRAVE:
            case CYCLOPE2_ERREUR_CODEUR_ABSOLU:
            case CYCLOPE2_ERREUR_MOTEUR:
            case CYCLOPE2_ERREUR_INCONNUE:
                fatal = true;
        }
        cout << "Trial = " + OttoUtils::numberToString(trial) << endl; /// à enlever
        if(trial > 5) fatal = true;

        // Abort sequence if this is fatal
        if (fatal) {theSequencer.resetM_StatusMatrix(); AbortSequence(-1); return;}

        // Else we have a non-fatal (or not yet fatal) error. Wait and retry.
        Sleep(5000);
    }
}


void BgEndAcq(Ottomator& theSequencer)
{
    bool fatal(false), problemSolved(false);
    int trial(0);
    string s;
    while (1) {
        DWORD ErrorCode = theSequencer.RemoveSample(isSampleIn); /// theSequencer à enlever
        if (!ErrorCode) return; // everything is ok, go for it

        // Display
        s = "Cyclope2: " + theSequencer.GetErrorText(ErrorCode, Language());
        ReportError(3, s.c_str());

        // Manage error

        switch(ErrorCode)
        {
            case CYCLOPE2_SERVO_OFF:
                problemSolved = theSequencer.solveCYCLOPE2_SERVO_OFF();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_STOP_BIT:
                problemSolved = theSequencer.solveCYCLOPE2_STOP_BIT();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_ALARME_ACQUITTABLE:
                problemSolved = theSequencer.solveCYCLOPE2_ALARME_ACQUITTABLE();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_COLLISION_PINCE:
                theSequencer.solveCYCLOPE2_COLLISION_PINCE();
            case CYCLOPE2_COLLISION_CHATEAU:
            case CYCLOPE2_DANGER_TIME_OUT:
            case CYCLOPE2_ALARME_GRAVE:
            case CYCLOPE2_ERREUR_CODEUR_ABSOLU:
            case CYCLOPE2_ERREUR_MOTEUR:
            case CYCLOPE2_ERREUR_INCONNUE:
                fatal = true;
        }
        cout << "Trial = " + OttoUtils::numberToString(trial) << endl; /// à enlever
        if(trial > 5) fatal = true;

        // Abort sequence if this is fatal
        if (fatal) {theSequencer.resetM_StatusMatrix(); AbortSequence(-1); return;}

        // Else we have a non-fatal (or not yet fatal) error. Wait and retry.
        Sleep(5000);
    }
}

void BgFinishCycle(Ottomator& theSequencer)
{
    bool fatal(false), problemSolved(false);
    int trial(0);
    string s;
    while (1) {
        DWORD ErrorCode = theSequencer.FinishCycle(); /// theSequencer à enlever
        if (!ErrorCode) return; // everything is ok, go for it

        // Display
        s = "Cyclope2: " + theSequencer.GetErrorText(ErrorCode, Language());
        ReportError(3, s.c_str());

        // Manage error

        switch(ErrorCode)
        {
            case CYCLOPE2_SERVO_OFF:
                problemSolved = theSequencer.solveCYCLOPE2_SERVO_OFF();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_STOP_BIT:
                problemSolved = theSequencer.solveCYCLOPE2_STOP_BIT();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_ALARME_ACQUITTABLE:
                problemSolved = theSequencer.solveCYCLOPE2_ALARME_ACQUITTABLE();
                if(!problemSolved) trial++; else trial = 0;
                break;
            case CYCLOPE2_COLLISION_PINCE:
                theSequencer.solveCYCLOPE2_COLLISION_PINCE();
            case CYCLOPE2_COLLISION_CHATEAU:
            case CYCLOPE2_DANGER_TIME_OUT:
            case CYCLOPE2_ALARME_GRAVE:
            case CYCLOPE2_ERREUR_CODEUR_ABSOLU:
            case CYCLOPE2_ERREUR_MOTEUR:
            case CYCLOPE2_ERREUR_INCONNUE:
                fatal = true;
        }
        cout << "Trial = " + OttoUtils::numberToString(trial) << endl; /// à enlever
        if(trial > 5) fatal = true;

        // Abort sequence if this is fatal
        if (fatal) {theSequencer.resetM_StatusMatrix(); AbortSequence(-1); return;}

        // Else we have a non-fatal (or not yet fatal) error. Wait and retry.
        Sleep(5000);
    }
}



/////////////////////////////////////////////////////////////////////////////



void sequenceMenu(Ottomator& theSequencer)
{
    char command(0);
    int sampleN(0), fetch(0), run(0), destination(1);
    Robot& R(theSequencer.m_robot);
    vector<string> messages;
    string binMessage;
    int message;

    do
    {
        // Display list of commands
        cout << "************************************" << endl;
        cout << "B - Please select a sequence to run:" << endl;
        cout << "************************************" << endl;
        cout << "1 - [demo] basic pick sample " << endl;
        cout << "2 - [demo] basic put sample " << endl;
        cout << "3 - [demo] go to sample N " << endl;
        cout << "4 - [demo] basic fetch/putback sample N " << endl;
        cout << "5 - [demo] complete cycle" << endl;
        cout << "6 - [CYCLOPE2] BgStartAcq()" << endl;
        cout << "7 - [CYCLOPE2] BgEndAcq()" << endl;
        cout << "8 - [CYCLOPE2] BgFinishCycle()" << endl;
        cout << "9 - [CYCLOPE2] Sequence reset" << endl;
        cout << "0 - back" << endl;
        cout << "Your choice is: " ;
        cin >> command;

        switch(command)
        {
            // 1 - [demo] basic pick sample
            case '1':
                basicPickSample(R);
            break;
            // 2 - [demo] basic put sample
            case '2':
                basicPutSample(R, 2);
                break;
            // 3 - [demo] go to sample N
            case '3':
                sampleN = input(OttoUtils::extendedSampleNb);
                (sampleN == POSITION_DETECTOR) ? goToCastel(theSequencer) : goToSample(sampleN, theSequencer);
                break;
            // 4 - [demo] basic fetch/putback sample N
            case '4':
                fetch = input(OttoUtils::fetchPutback);
                sampleN = input(OttoUtils::sampleNb);
                cout << "********** Lift up **********" << endl;
                R.setPosition(3, 1);
                // Motion 1
                (fetch) ? goToSample(sampleN, theSequencer) : goToCastel(theSequencer);
                cout << "********** Basic Pick **********" << endl;
                basicPickSample(R);
                // Motion 2
                (fetch) ? goToCastel(theSequencer) : goToSample(sampleN, theSequencer);
                cout << "********** Basic Put **********" << endl;
                (fetch) ? destination = 3 : destination = 2; basicPutSample(R, destination);
                // Finish
                (fetch) ? goToSample(22, theSequencer) : goToSample(0, theSequencer);
                break;
            // 5 - [demo] complete cycle
            case '5':
                run = 0;
                // Overwrite
                messages = theSequencer.getM_sampleArray();
                if(messages.size() != 0) if(overWriteM_sampleArray(messages)) inputSampleArray(theSequencer);
                do
                {   // Run cycler
                    message = theSequencer.cycler();
                    if(message < 0)
                    {
                        cout << "The message is: " << message << endl;
                        run = 1;
                    } else
                    {
                        cout << "                SV\t" << "STP\t" << "EMGS\t" << "EMGV\t" << "LAT\t" << "P1\t" << "P2\t" << "ALML\t"
                             << "ALMH*\t" << "ABER*\t" << "MOTO*\t" << "XCAS*\t" << endl;
                        cout << "The message is: ";
                        binMessage = OttoUtils::decToBin(message, true);
                        for(unsigned int i=0; i<12; ++i)
                        {
                            (i < binMessage.length()) ? cout << binMessage[i] : cout << "0";
                            cout << "\t";
                        }
                        cout << endl;
                        run = 0;
                    }
                    if(message == Need_input)
                    {   // input
                        inputSampleArray(theSequencer);
                        run = 1;
                    } else if(message == Fetch_completed)
                    {   // wait
                        run = measuringFinished();
                    } else if(message == Finish_completed)
                    {   // finish
                        run = 0;
                    } else
                    {   // error
                        cout << "Main > [ERROR] please solve above issues" << endl;
                        run = 0;
                    }
                } while(run);
                break;
            // 6 -
            case '6':
                CurrentPosition = input(OttoUtils::sampleNb);
                BgStartAcq(theSequencer);
                break;
            // 7 -
            case '7':
                BgEndAcq(theSequencer);
                break;
            // 8 -
            case '8':
                BgFinishCycle(theSequencer);
                break;
            // 9 -
            case '9':
                theSequencer.resetM_StatusMatrix();
                cout << "Sequence reset" << endl;
                break;
            case '0':
                command = 0;
                break;
            default:
                cout << "Main > [Error] Invalid command" << endl;
        }
    }
    while (command != 0);
}

void convertMe()
{
    int a;
    cin >> a;
    string b = OttoUtils::decToBin(a, true);
    cout << " = " << b << endl;
}


int main()
{
    cout << "-----------------------------------" << endl;
    cout << "| Ottomator100 InterWinner Edition|" << endl;
    cout << "-----------------------------------" << endl;


    // Create objects
    Ottomator theSequencer;

    // Connect to serial port
    cout << "Now you have to connect to the right serial port. ";
    connectToMobus(theSequencer);

    theSequencer.m_robot.m_busManager.modbusWriteData(OttoUtils::vAct, 0x6, 0x0001, 0x00); /// Secure motion stop

    char command(0);
    do
    {
        // Display list of commands
        cout << "***************************************" << endl;
        cout << "***************************************" << endl;
        cout << "               MAIN MENU               " << endl;
        cout << "***************************************" << endl;
        cout << "***************************************" << endl;
        cout << "A - Elementary commands... " << endl;
        cout << "B - Sequences... " << endl;
        cout << "C - Convert Me " << endl;
        cout << "D - Coordinates " << endl;
        cout << "E - eeeee " << endl;
        cout << "F - quit " << endl;
        cout << "Your choice is: " ;
        cin >> command;

        command = toupper(command);

        switch(command)
        {
            //
            case 'A':
                elementaryMenu(theSequencer);
                break;
            //
            case 'B':
                sequenceMenu(theSequencer);
                break;
            //
            case 'C':
                convertMe();
                break;
            //
            case 'D':
                for(int n=0; n < 23; ++n)
                {
                    cout << "Sample " << n << "\t x = " << theSequencer.xOfSample(n) << "\t y = " << theSequencer.yOfSample(n) << endl;
                }
                break;
            //
            case 'E':
                theSequencer.solveCYCLOPE2_COLLISION_PINCE();

                break;
            //
            case 'F':
                command = 0;
                break;
            default:
                cout << "Main > [Error] Invalid command" << endl;
        }
    }
    while (command != 0);


    return 0;
}

