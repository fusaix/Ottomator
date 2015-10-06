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
                theSequencer.m_robot.m_busManager.modbusReadData(OttoUtils::pAct, 0x03, 0x9000, 2, 10, 2);
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
    bool run(0);
    cout << "You can know measure..." << endl;
    cout << "Measuring finished? (1) / Abort? (0) ";
    cin >> run;
    return run;
}



void sequenceMenu(Ottomator& theSequencer)
{
    char command(0);
    int sampleN(0), fetch(0), run(0), destination(1);
    Robot& R(theSequencer.m_robot);
    vector<string> messages;
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
        cout << "6 -  " << endl;
        cout << "7 -  " << endl;
        cout << "8 - " << endl;
        cout << "9 -  " << endl;
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
                    messages = theSequencer.cycler();
                    for (unsigned int i = 0; i < messages.size(); ++i)
                        (i%2) ? cout << messages[i] << ", " : cout << messages[i] << " ";
                    cout << endl;
                    if(messages[0] == "Need input")
                    {   // input
                        inputSampleArray(theSequencer);
                        run = 1;
                    } else if(messages[0] == "Fetch completed")
                    {   // wait
                        run = measuringFinished();
                    } else if(messages[0] == "Finish completed")
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

                break;
            // 7 -
            case '7':

                break;
            // 8 -
            case '8':

            // 9 -
            case '9':
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
                theSequencer.resetM_StatusMatrix();
                theSequencer.displayM_StatusMatrix();
                theSequencer.setNCurrentSample(1);
                theSequencer.displayM_StatusMatrix();
                theSequencer.m_robot.updateStatusOfCurrent(1);
                //theSequencer.m_robot.m_busManager.modbusReadData(1, 0x3, 0x9005, 0x0001);

                theSequencer.displayM_StatusMatrix();

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

