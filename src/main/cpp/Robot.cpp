// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <iostream>
#include <time.h>

void Robot::RobotInit()
{
    m_gearboxGauche.disableVoltageCompensation();
    m_gearboxDroite.disableVoltageCompensation();

    m_gearboxGauche.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_gearboxDroite.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    m_gyro.Calibrate();

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    frc::Shuffleboard::GetTab("voltage").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);

    m_gearboxDroite.setInverted(false);
    m_gearboxGauche.setInverted(true);


    characterization.initializeTestData();


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

void Robot::RobotPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::TeleopInit()
{
    m_gearboxDroite.resetExternalEncodeur();
    m_gearboxGauche.resetExternalEncodeur();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

// Robot::Robot(){
// }

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
    m_gearboxGauche.setSpeed(m_va_left);
    m_gearboxDroite.setSpeed(m_va_right);
}

void Robot::TeleopPeriodic()
{
    Drive(-m_joystick.getY(), m_joystick.getX());

    if (m_joystick.getBButtonPressed() && (!m_override))
    {
        characterization.nextTest();
    }

    if (m_joystick.getXButtonPressed() && (!m_override))
    {
        characterization.previousTest();
    }

    if (m_joystick.getAButtonPressed())
    {
        m_override = !m_override;

        m_time0 = std::time(0);

        if (m_override)
        {
            characterization.stopTest(&m_gearboxGauche, &m_gearboxDroite);
        }
        else
        {
            characterization.startTest();
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

        characterization.logData(&m_gearboxGauche, &m_gearboxDroite, &m_gyro, m_ramp);
    }

    if (m_joystick.getYButtonPressed())
    {
        m_isLogging = !m_isLogging;

        m_time0 = std::time(0);

        if (m_isLogging)
        {
            characterization.freeDriveLog(&m_gearboxGauche, &m_gearboxDroite);
        }
        else
        {
            characterization.deleteLogFileDriving();
        }
    }

    if (m_isLogging)
    {
        characterization.logData(&m_gearboxGauche, &m_gearboxDroite, &m_gyro, m_ramp);
    }
}