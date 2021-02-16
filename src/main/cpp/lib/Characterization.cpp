#include "lib/Characterization.h"


Characterization::Characterization(){
    m_LogFilename = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
}
void Characterization::initializeTestData() {
    int i;
    // Low Voltages
    for (i = 0; i < TEST_LOWVOLTAGE_NB; i++) {
        TestData[i * 2].m_voltage = TEST_LOWVOLTAGE_MAX * (double) (i + 1) / (double) TEST_LOWVOLTAGE_NB;
        TestData[i * 2].m_flags = 0;

        TestData[i * 2 + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX * (double) (i + 1) / (double) TEST_LOWVOLTAGE_NB);
        TestData[i * 2 + 1].m_flags = 0;
    }
    // Medium Voltages
    for (i = 0; i < TEST_MEDIUMVOLTAGE_NB; i++) {
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_voltage = TEST_LOWVOLTAGE_MAX +
                                                           (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) *
                                                           (double) (i + 1) / (double) TEST_MEDIUMVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX +
                                                                 (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) *
                                                                 (double) (i + 1) / (double) TEST_MEDIUMVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    // High Voltages
    for (i = 0; i < TEST_HIGHVOLTAGE_NB; i++) {
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_voltage = TEST_MEDIUMVOLTAGE_MAX +
                                                                                   (TEST_HIGHVOLTAGE_MAX -
                                                                                    TEST_MEDIUMVOLTAGE_MAX) *
                                                                                   (double) (i + 1) /
                                                                                   (double) TEST_HIGHVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_voltage = -(TEST_MEDIUMVOLTAGE_MAX +
                                                                                         (TEST_HIGHVOLTAGE_MAX -
                                                                                          TEST_MEDIUMVOLTAGE_MAX) *
                                                                                         (double) (i + 1) /
                                                                                         (double) TEST_HIGHVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    Message = frc::Shuffleboard::GetTab("voltage").Add("Message", "").WithWidget(
            frc::BuiltInWidgets::kTextView).GetEntry();
}

void Characterization::waitingMessageTests() {
    char txt[256];
    if (ISFLAG_OFF(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done)) {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En Attente ... Appuyer sur A pour Démarrer.", CurrentTestID,
                TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    } else {
        sprintf(txt,
                "TEST %d / %d [ %.2f Volts ]. DEJA EFFECTUE ! ... Appuyer sur A pour Démarrer à nouveau.( Le précédent LOGFILE sera conservé.)",
                CurrentTestID, TEST_TOTAL_NB, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
}

void Characterization::allFinishedMessageTests() {
    char txt[256];
    sprintf(txt, "%d / %d  TESTS EFFECTUES ! ", TEST_TOTAL_NB * 2, TEST_TOTAL_NB * 2);
    Message.SetString(txt);
}

void Characterization::inProgressMessageTests() {
    char txt[256];
    sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En cours ... Appuyer à nouveau sur A pour arrêter.", CurrentTestID,
            TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
    Message.SetString(txt);
}

void Characterization::messageTestEnAttente()
{
    char txt[256];
    if (ISFLAG_OFF(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done))
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En Attente ... Appuyer sur A pour Démarrer.", CurrentTestID, TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
    else
    {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. DEJA EFFECTUE ! ... Appuyer sur A pour Démarrer à nouveau.( Le précédent LOGFILE sera conservé.)", CurrentTestID, TEST_TOTAL_NB, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
}

void Characterization::messageTestTousEffectues()
{
    char txt[256];
    sprintf(txt, "%d / %d  TESTS EFFECTUES ! ", TEST_TOTAL_NB * 2, TEST_TOTAL_NB * 2);
    Message.SetString(txt);
}

void Characterization::messageTestEnCours()
{
    char txt[256];
    sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En cours ... Appuyer à nouveau sur A pour arrêter.", CurrentTestID, TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
    Message.SetString(txt);
}

void Characterization::nextTest(){
            CurrentTestID += 1;
        if (CurrentTestID < TEST_TOTAL_NB * 2)
        {
            messageTestEnAttente();
        }
        else
        {
            messageTestTousEffectues();
        }
}

void Characterization::previousTest(){
        CurrentTestID -= 1;
        if (CurrentTestID < TEST_TOTAL_NB * 2)
        {
            messageTestEnAttente();
        }
        else
        {
            messageTestTousEffectues();
        }
}

void Characterization::startTest(){
    CurrentTestID += 1;
    if (CurrentTestID < TEST_TOTAL_NB * 2)
    {
        delete m_LogFile;
        messageTestEnAttente();
    }
    else
    {
        messageTestTousEffectues();
    }
}

void Characterization::stopTest(Gearbox *gearboxGauche, Gearbox *gearboxDroite){
    if (CurrentTestID < TEST_TOTAL_NB * 2)
    {
        FLAG_ON(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done);
        messageTestEnCours();

        char prefix[256];
        if (TestData[CurrentTestID].m_voltage < 0)
        {
            sprintf(prefix, "/home/lvuser/logs/-test_%d_%.2fvolts_", CurrentTestID, TestData[CurrentTestID].m_voltage);
        }
        else
        {
            sprintf(prefix, "/home/lvuser/logs/+test_%d_%.2fvolts_", CurrentTestID, TestData[CurrentTestID].m_voltage);
        }

        m_LogFile = new CSVLogFile(prefix, "Right", "Left", "neoD1", "neoD2", "neoG1", "neoG2", "gyro", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
        m_LogFilename.SetString(m_LogFile->GetFileName());

        gearboxDroite->resetExternalEncodeur();
        gearboxGauche->resetExternalEncodeur();
    }
}

void Characterization::logData(Gearbox *gearboxGauche, Gearbox *gearboxDroite, frc::ADXRS450_Gyro *gyro, double ramp){
    m_LogFile->Log(gearboxDroite->getExternalEncoderDistance(), gearboxGauche->getExternalEncoderDistance(), gearboxDroite->getInternalEncoderPosition(0), gearboxDroite->getInternalEncoderPosition(1), gearboxDroite->getInternalEncoderPosition(0), gearboxDroite->getInternalEncoderPosition(1), gyro->GetAngle(), TestData[CurrentTestID].m_voltage, gearboxDroite->getBusVoltage(0), gearboxDroite->getBusVoltage(1), gearboxGauche->getBusVoltage(0), gearboxGauche->getBusVoltage(1), gearboxDroite->getAppliedOutput(0), gearboxDroite->getAppliedOutput(1), gearboxGauche->getAppliedOutput(0), gearboxGauche->getAppliedOutput(1), gearboxDroite->getOutputCurrent(0), gearboxDroite->getOutputCurrent(1), gearboxGauche->getOutputCurrent(0), gearboxGauche->getOutputCurrent(1), ramp);

}

void Characterization::deleteLogFileDriving(){
    delete m_LogFileDriving;
}

void Characterization::freeDriveLog(Gearbox *gearboxGauche, Gearbox *gearboxDroite){
    m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "Right", "Left", "neoD1", "neoD2", "neoG1", "neoG2", "gyro", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
    m_LogFilenameDriving.SetString(m_LogFileDriving->GetFileName());
    gearboxGauche->resetExternalEncodeur();
    gearboxDroite->resetExternalEncodeur();
}

void Characterization::setSpeedDriveOld(Gearbox *gearboxGauche, Gearbox *gearboxDroite){
    gearboxGauche->setSpeed(TestData[CurrentTestID].m_voltage / gearboxGauche->getBusVoltage(0), 0);
    gearboxGauche->setSpeed(TestData[CurrentTestID].m_voltage / gearboxGauche->getBusVoltage(1), 1);
    gearboxDroite->setSpeed(TestData[CurrentTestID].m_voltage / gearboxGauche->getBusVoltage(0), 0);
    gearboxDroite->setSpeed(TestData[CurrentTestID].m_voltage / gearboxGauche->getBusVoltage(1), 1);
}

void Characterization::logStateSwitch(Gearbox *gearboxGauche, Gearbox *gearboxDroite, double *m_time0, double *m_ramp){
    switch (m_logState)
        {
        case 1:
            m_LogFile = new CSVLogFile(m_prefix, "encoderGetD", "encoderGetG", "encoderGetRawD", "encoderGetRawG", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
            m_LogFilename.SetString(m_LogFile->GetFileName());
            gearboxDroite->resetExternalEncodeur();
            gearboxGauche->resetExternalEncodeur();
            m_logState = 2;
            break;

        case 2:
            if (std::time(0) - *m_time0 < TIME_RAMP)
            {
                *m_ramp = 1;
            }
            else
            {
                *m_ramp = 0;
            }
            m_LogFile->Log( gearboxDroite->getExternalEncoder(),
                            gearboxGauche->getExternalEncoder(),
                            gearboxDroite->getExternalEncoderRaw(),
                            gearboxGauche->getExternalEncoderRaw(),
                            TestData[CurrentTestID].m_voltage,
                            gearboxDroite->getBusVoltage(0),
                            gearboxDroite->getBusVoltage(1),
                            gearboxGauche->getBusVoltage(0),
                            gearboxGauche->getBusVoltage(1),
                            gearboxDroite->getAppliedOutput(0),
                            gearboxDroite->getAppliedOutput(1),
                            gearboxGauche->getAppliedOutput(0),
                            gearboxGauche->getAppliedOutput(1),
                            gearboxDroite->getOutputCurrent(0),
                            gearboxDroite->getOutputCurrent(1),
                            gearboxGauche->getOutputCurrent(0),
                            gearboxGauche->getOutputCurrent(1),
                            m_ramp);
            break;

        case 3:
            delete m_LogFile;
            m_logState = 0;
            break;

        default:
            break;
        }
}