// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <time.h>


#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

// #define AMAX 5.1 // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX                       \
    (((2.0 * VMAX) / TRACKWIDTH) / \
     1.7) // vitesse angulaire Max theorique	.. à modifier avec Garice

// Flags Manipulation
// #define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
// #define FLAG_OFF(val, flag) ((val) &= ~(flag))
// #define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!                              // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
// #define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
// #define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
// #define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO

// #define VOLTAGE_COMPENSATION_VALUE 11.5
// TEST *********************************************
// #define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
// #define TEST_LOWVOLTAGE_MAX 0.15 // Volts

// #define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
// #define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts

// #define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
// #define TEST_HIGHVOLTAGE_MAX 12.0 // Volts

// #define TEST_TOTAL_NB (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + TEST_HIGHVOLTAGE_NB)



#define TIME_RAMP 0





void Robot::RobotInit()
{
    m_gearboxGauche.disableVoltageCompensation();
    m_gearboxDroite.disableVoltageCompensation();

    m_gearboxGauche.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_gearboxDroite.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_gyro.Calibrate();

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFilename = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    frc::Shuffleboard::GetTab("voltage").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);

    m_gearboxDroite.setInverted(false);
    m_gearboxGauche.setInverted(true);


    tests.initializeTestData();


    m_gearboxGauche.setMotorCoefficients(false);
    m_gearboxDroite.setMotorCoefficients(false);


    m_va_max.m_speed = 3;
    m_va_max.m_acceleration = 5;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;
}


void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_gearboxDroite.resetExternalEncodeur();
    m_gearboxGauche.resetExternalEncodeur();
}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

Robot::Robot(){
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<Robot>();
}

#endif






void Robot::Drive(double jy, double jx)
{
    jy = Deadband(jy, 0.1);
    jx = Deadband(jx, 0.2);
    std::cout << jy << "       " << jx << std::endl;

    m_joystick.getSpeedsAndAccelerations(&m_va_left, &m_va_right, &m_va_max, jx, jy);
    m_gearboxGauche.setSpeed(&m_va_left);
    m_gearboxDroite.setSpeed(&m_va_right);
}




void Robot::TeleopPeriodic()
{
    Drive(-m_joystick.getY(), m_joystick.getX());

    if (m_joystick.getBButtonPressed() && (!m_override))
    {
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

    if (m_driverController.GetXButtonPressed() && (!m_override))
    {
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

    if (m_driverController.GetAButtonPressed())
    {
        m_override = !m_override;

        m_time0 = std::time(0);

        if (m_override)
        {
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
                m_encodeurExterneDroite.Reset();
                m_encodeurExterneGauche.Reset();
            }
        }
        else
        {
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
    }

    if (m_override)
    {
        if (std::time(0) - m_time0 < TIME_RAMP)
        {
            m_ramp = 1;
        }
        else
        {
            m_ramp = 0;
        }
        double md0, md1, mg0, mg1;
        md0 = m_moteurDroite.GetBusVoltage() * m_moteurDroite.GetAppliedOutput();
        md1 = m_moteurDroiteFollower.GetBusVoltage() * m_moteurDroiteFollower.GetAppliedOutput();
        mg0 = m_moteurGauche.GetBusVoltage() * m_moteurGauche.GetAppliedOutput();
        mg1 = m_moteurGaucheFollower.GetBusVoltage() * m_moteurGaucheFollower.GetAppliedOutput();

        m_LogFile->Log(m_encodeurExterneDroite.GetDistance(), m_encodeurExterneGauche.GetDistance(), m_encodeurDroite1.GetPosition(), m_encodeurDroite2.GetPosition(), m_encodeurGauche1.GetPosition(), m_encodeurGauche2.GetPosition(), m_gyro.GetAngle(), TestData[CurrentTestID].m_voltage, m_moteurDroite.GetBusVoltage(), m_moteurDroiteFollower.GetBusVoltage(), m_moteurGauche.GetBusVoltage(), m_moteurGaucheFollower.GetBusVoltage(), m_moteurDroite.GetAppliedOutput(), m_moteurDroiteFollower.GetAppliedOutput(), m_moteurGauche.GetAppliedOutput(), m_moteurGaucheFollower.GetAppliedOutput(), m_moteurDroite.GetOutputCurrent(), m_moteurDroiteFollower.GetOutputCurrent(), m_moteurGauche.GetOutputCurrent(), m_moteurGaucheFollower.GetOutputCurrent(), m_ramp);
    }

    if (m_driverController.GetYButtonPressed())
    {
        m_isLogging = !m_isLogging;

        m_time0 = std::time(0);

        if (m_isLogging)
        {
            m_LogFileDriving = new CSVLogFile("/home/lvuser/logs/freeRiding", "Right", "Left", "neoD1", "neoD2", "neoG1", "neoG2", "gyro", "Theorical Voltage", "BusVoltageD1", "BusVoltageD2", "BusVoltageG1", "BusVoltageG2", "AppliedOutputD1", "AppliedOutputD2", "AppliedOutputG1", "AppliedOutputG2", "currentD1", "currentD2", "currentG1", "currentG2", "rampActive");
            m_LogFilenameDriving.SetString(m_LogFileDriving->GetFileName());
            m_encodeurExterneDroite.Reset();
            m_encodeurExterneGauche.Reset();
        }
        else
        {
            delete m_LogFileDriving;
        }
    }

    if (m_isLogging)
    {
        m_LogFileDriving->Log(m_encodeurExterneDroite.GetDistance(), m_encodeurExterneGauche.GetDistance(), m_encodeurDroite1.GetPosition(), m_encodeurDroite2.GetPosition(), m_encodeurGauche1.GetPosition(), m_encodeurGauche2.GetPosition(), m_gyro.GetAngle(), TestData[CurrentTestID].m_voltage, m_moteurDroite.GetBusVoltage(), m_moteurDroiteFollower.GetBusVoltage(), m_moteurGauche.GetBusVoltage(), m_moteurGaucheFollower.GetBusVoltage(), m_moteurDroite.GetAppliedOutput(), m_moteurDroiteFollower.GetAppliedOutput(), m_moteurGauche.GetAppliedOutput(), m_moteurGaucheFollower.GetAppliedOutput(), m_moteurDroite.GetOutputCurrent(), m_moteurDroiteFollower.GetOutputCurrent(), m_moteurGauche.GetOutputCurrent(), m_moteurGaucheFollower.GetOutputCurrent(), m_ramp);
    }

    if (m_driverController.GetYButton())
    {

        m_moteurCoveyor.Set(1);
        m_moteurFeeder.Set(1);
    }
    else
    {
        m_moteurCoveyor.Set(0);
        m_moteurFeeder.Set(0);
    }


    if (m_driverController.GetPOV(0))
    {
        if (modeClimberJF)
        {
            m_moteurTreuil.Set(-0.3);
        }
        else
        {
            m_moteurTreuil.Set(0);
        }
    }
    else if (m_driverController.GetPOV(90))
    {
        if (!modeClimberJF)
        {
            m_moteurTreuil.Set(0);
        }
    }
    else if (m_driverController.GetPOV(180))
    {
        m_moteurTreuil.Set(0.3);
    }
    else
    {
        m_moteurTreuil.Set(0);
    }
}



//the old version of old
// void Robot::DriveOld(double forward, double turn)
// {
//     if (!m_override)
//     {
//         forward = Deadband(forward);
//         turn = Deadband(turn, 0.2);


//         double v = forward * VMAX;
//         double w = turn * WMAX;


//         double lwheel = v + (w * HALF_TRACKWIDTH);
//         double rwheel = v - (w * HALF_TRACKWIDTH);

//         double k;
//         k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
//         lwheel *= k;
//         rwheel *= k;

//         m_moteurGauche.Set(lwheel);
//         m_moteurGaucheFollower.Set(lwheel);
//         m_moteurDroite.Set(rwheel);
//         m_moteurDroiteFollower.Set(rwheel);
//     }
//     else
//     {
//         m_moteurGauche.Set(TestData[CurrentTestID].m_voltage / m_moteurGauche.GetBusVoltage());
//         m_moteurGaucheFollower.Set(TestData[CurrentTestID].m_voltage / m_moteurGaucheFollower.GetBusVoltage());
//         m_moteurDroite.Set(TestData[CurrentTestID].m_voltage / m_moteurDroite.GetBusVoltage());
//         m_moteurDroiteFollower.Set(TestData[CurrentTestID].m_voltage / m_moteurDroiteFollower.GetBusVoltage());
//     }
// }