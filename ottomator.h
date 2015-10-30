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

// Reserved for debug and futur development
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

// Ottomator Error Codes
#define SV      1
#define STP     2
#define EMGS    4
#define EMGV    8
#define LAT     16
#define ALML    32
#define P1      64
#define P2      128
#define ALMH    256
#define ABER    512
#define MOTO    1024
#define XCAS    2048
#define TIME    4096

// Cyclope2 Error Codes
#define CYCLOPE2_FENETRE_OUVERTE            1 // wait
#define CYCLOPE2_PORTE_CHATEAU_OUVERTE      2 // wait
#define CYCLOPE2_SERVO_OFF                  4 // servo on
#define CYCLOPE2_STOP_BIT                   8 // go
#define CYCLOPE2_ALARME_ACQUITTABLE         16  // reset alarm
#define CYCLOPE2_COLLISION_PINCE            32  // solve unavailable position error, then abort
#define CYCLOPE2_COLLISION_CHATEAU          64  // abort
#define CYCLOPE2_DANGER_TIME_OUT            128 // abort
#define CYCLOPE2_ALARME_GRAVE               256 // abort
#define CYCLOPE2_ERREUR_CODEUR_ABSOLU       512 // abort
#define CYCLOPE2_ERREUR_MOTEUR              1024 // abort
#define CYCLOPE2_ERREUR_INCONNUE            2048 // abort

/*
static const std::string errorSymbols[]={"SV : Un servomoteur n'est pas activé. \n",
                                         "STP : Le passeur est en mode pause. \n",
                                         "EMGS : Les cartes d'axes ont détecté un arrêt d'urgence. Vérifiez que les fenêtres sont bien fermée puis réarmez le passeur. \n",
                                         "EMGV : La carte du vérin a détecté un arrêt d'urgence. Vérifiez que les fenêtres sont bien fermée puis réarmez le passeur. \n",
                                         "LAT : La porte latérale du château n'est pas fermée. Veuillez la fermer. \n",
                                         "ALML : Les cartes d'axe retourne une alarme. Elle va être automatiquement acquittée par m_robot.alarmReset(int actuator). \n",
                                         "P1 : Le capteur de pince #1 est enclenché. Veuillez intervenir pour réarmer manuellement le passeur. \n",
                                         "P2 : Le capteur de pince #2 est enclenché. Veuillez intervenir pour réarmer manuellement le passeur. \n",
                                         "ALMH : Les cartes d'axe retourne une alarme nécessitant le redémarrage du système. \n",
                                         "ABER : Les cartes d'axe ont détecté une erreur de positionnement absolu. Il est possible que des câbles ont été déconnecté. \n",
                                         "MOTO : Défaut moteur sur le vérin. Veuillez vérifier l'intégrité du château de plomb. \n",
                                         "XCAS : L'axe X a heurté le château. La trappe actionnée par le vérin est sans doute restée fermée. Veuillez vérifier l'intégrité des axes. \n"};
*/

class Ottomator
{
public:
    Ottomator();
    ~Ottomator();

    //////////////////////////////// HIGH LEVEL STUFF ///////////////////////////////////////

    //virtual DWORD InsertSample(int iSample);  // Does the initialization/birth if necessary. Returns 0 on ok.
    //virtual DWORD RemoveSample(int iSample);  // Does the initialization/birth if necessary. Returns 0 on ok.
    //virtual DWORD FinishCycle();  // Does the necessary to finish the cycle. Returns 0 on ok.
    DWORD InsertSample(int iSample);  // Does the initialization/birth if necessary. Returns 0 on ok.
    DWORD RemoveSample(int iSample);  // Does the initialization/birth if necessary. Returns 0 on ok.
    DWORD FinishCycle();  // Does the necessary to finish the cycle. Returns 0 on ok.

    //virtual std::string GetErrorText(int input, int iLanguage);
    std::string GetErrorText(int ErrorNum, int iLanguage);


    // Problem solvers
    //virtual void solveCYCLOPE2_COLLISION_PINCE(); // encountered CYCLOPE2_COLLISION_PINCE, thus open the plier and get the Z up. After that, you have to manage it as a fatal error.
    //virtual bool solveCYCLOPE2_ALARME_ACQUITTABLE(); // CYCLOPE2_ALARME_ACQUITTABLE: Try to reset all low alarms, if unable to reset all low alarms after several trials then you have to manage it as a fatal error.
    //virtual bool solveCYCLOPE2_STOP_BIT(); // CYCLOPE2_STOP_BIT: Clear STP bit for all actuators.
    //virtual bool solveCYCLOPE2_SERVO_OFF();
    void solveCYCLOPE2_COLLISION_PINCE(); // encountered CYCLOPE2_COLLISION_PINCE, thus open the plier and get the Z up. After that, you have to manage it as a fatal error.
    bool solveCYCLOPE2_ALARME_ACQUITTABLE(); // CYCLOPE2_ALARME_ACQUITTABLE: Try to reset all low alarms, if unable to reset all low alarms after several trials then you have to manage it as a fatal error.
    bool solveCYCLOPE2_STOP_BIT(); // CYCLOPE2_STOP_BIT: Clear STP bit for all actuators.
    bool solveCYCLOPE2_SERVO_OFF();


    //////////////////////////////// LOW LEVEL STUFF ////////////////////////////////////////

    // Utilities
    int Cyclope2_errorCode(int message);
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
