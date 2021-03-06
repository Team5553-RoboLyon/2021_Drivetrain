// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
    m_isLogging = false;

    m_gearboxGauche.restoreFactoryDefaults();
    m_gearboxDroite.restoreFactoryDefaults();

    m_gearboxGauche.setOpenLoopRampRate(TIME_RAMP);
    m_gearboxDroite.setOpenLoopRampRate(TIME_RAMP);


    m_gearboxGauche.disableVoltageCompensation();
    m_gearboxDroite.disableVoltageCompensation();

    m_gearboxGauche.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_gearboxDroite.setIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // m_gyro.Calibrate();

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    // frc::Shuffleboard::GetTab("voltage").Add(m_gyro).WithWidget(frc::BuiltInWidgets::kGyro);

    m_gearboxDroite.setInverted(false);
    m_gearboxGauche.setInverted(true);


    characterization.initializeTestData();


    m_gearboxGauche.setMotorCoefficients(false);
    m_gearboxDroite.setMotorCoefficients(false);

    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("voltage").Add("Data", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();

#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif


    m_gearboxDroite.setDistancePerPulse(1);
    m_gearboxGauche.setDistancePerPulse(1);
    m_gearboxDroite.setSamplesToAverage(65);
    m_gearboxGauche.setSamplesToAverage(65);

    m_gearboxGauche.setInverted(false);
    m_gearboxDroite.setInverted(true);

    sprintf(m_invertedPrefix, "L%d%dR%d%d", (int)m_gearboxGauche.getInverted(0), (int)m_gearboxGauche.getInverted(1), (int)m_gearboxDroite.getInverted(0), (int)m_gearboxDroite.getInverted(1));



    m_va_max.m_speed = VMAX;
    m_va_max.m_acceleration = AMAX;
    m_va_max.m_jerk = 5;

    m_va_left.m_speed = 0;
    m_va_left.m_acceleration = 0;
    m_va_right.m_speed = 0;
    m_va_right.m_acceleration = 0;


    m_gearboxDroite.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5, 0);
    m_gearboxDroite.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5, 0);
    m_gearboxDroite.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5, 1);
    m_gearboxDroite.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5, 1);

    m_gearboxGauche.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5, 0);
    m_gearboxGauche.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5, 0);
    m_gearboxGauche.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 5, 1);
    m_gearboxGauche.setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 5, 1);

    // Robot::AddPeriodic([&]() {characterization.logState(&m_gearboxGauche, &m_gearboxDroite, &m_time0, &m_ramp);}, 5_ms, 5_ms);
    Robot::AddPeriodic([&]() {
        DriveB();
        if (m_isLogging)
        {
            characterization.log(m_targetLeftSpeed,
                                  m_va_left.m_speed,
                                  m_gearboxGauche.getExternalEncoderDistance(),
                                  m_va_left.m_acceleration,
                                  m_gearboxGauche.getKv().getVoltage(2, &m_va_left),
                                  m_gearboxGauche.getBusVoltage(0) * m_gearboxGauche.getAppliedOutput(0),
                                  m_gearboxGauche.getBusVoltage(1) * m_gearboxGauche.getAppliedOutput(1),
                                  m_gearboxDroite.getBusVoltage(0) * m_gearboxGauche.getAppliedOutput(0),
                                  m_gearboxDroite.getBusVoltage(1) * m_gearboxGauche.getAppliedOutput(1),
                                  m_pdp.GetCurrent(0),
                                  m_pdp.GetCurrent(1),
                                  m_pdp.GetCurrent(14),
                                  m_pdp.GetCurrent(15),
                                  m_gearboxDroite.getOutputCurrent(0),
                                  m_gearboxDroite.getOutputCurrent(1),
                                  m_gearboxGauche.getOutputCurrent(0),
                                  m_gearboxGauche.getOutputCurrent(1),
                                  m_gearboxDroite.getFaults(0),
                                  m_gearboxDroite.getFaults(1),
                                  m_gearboxGauche.getFaults(0),
                                  m_gearboxGauche.getFaults(1)
            );
        }


    }, 5_ms, 5_ms);
}


void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::RobotPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::TeleopInit()
{
    characterization.newLogFile("/home/lvuser/logs/freeRiding", "target", "speed", "tureSpeed", "acceleration", "voltageTheoric", " trueVoltageG1 ", "trueVoltageG2", "trueVoltageR1", "trueVoltageR2", "pdp0", "pdp1", "pdp14", "pdp15", "A1", "A2", "A3", "A4", "erreur1", "erreur2", "erreur3", "erreur4");
    m_isLogging = true;
    m_gearboxDroite.resetExternalEncodeur();
    m_gearboxGauche.resetExternalEncodeur();
    #if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
    #endif
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::Drive(double forward, double turn)
{

    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.1);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    double target_left_speed = lwheel * VMAX;
    double target_right_speed = rwheel * VMAX;

    m_joystick.updateVelocityAndAcceleration(&m_va_left, &m_va_max, target_left_speed, 0.02);
    m_joystick.updateVelocityAndAcceleration(&m_va_right, &m_va_max, target_right_speed, 0.02);


    m_gearboxGauche.setSpeed(m_gearboxGauche.getKv().getVoltage(0, &m_va_left) / m_gearboxGauche.getBusVoltage(0), 0);
    m_gearboxGauche.setSpeed(m_gearboxGauche.getKv().getVoltage(1, &m_va_left) / m_gearboxGauche.getBusVoltage(1), 1);

    m_gearboxDroite.setSpeed(m_gearboxDroite.getKv().getVoltage(2, &m_va_right) / m_gearboxDroite.getBusVoltage(0), 0);
    m_gearboxDroite.setSpeed(m_gearboxDroite.getKv().getVoltage(3, &m_va_right) / m_gearboxDroite.getBusVoltage(1), 1);
    std::cout << forward << "          " << target_left_speed << "          " << m_va_left.m_speed << "   :   " << m_va_left.m_acceleration << "             " << (m_gearboxGauche.getKv().getVoltage(0, &m_va_left) +  m_gearboxDroite.getKv().getVoltage(0, &m_va_right)) / 2 << "      :     " << std::endl;
}

void Robot::TeleopPeriodic()
{
    #if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
    #endif

#if XBOX_CONTROLLER
    Drive(-m_joystick.getY(0), m_joystick.getX(1));
#else:
    m_va_max.m_acceleration = m_PowerEntry.GetDouble(0.0f);
    DriveA(-m_joystick.getY(0), m_joystick.getZ(1));
    std::cout << m_gearboxGauche.getExternalEncoderDistance() << std::endl;
#endif
#if XBOX_CONTROLLER
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

 #else
    m_turnAdjustFactor = (m_joystick.getThrottle(1) + 1.0) / 2.0;
    m_customEntry.SetDouble(m_turnAdjustFactor);
 #endif



    Drive(-m_joystick.getY(0), m_joystick.getX(1));

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

void Robot::DriveOld(double forward, double turn)
{
    if (!m_override)
    {
        forward = Deadband(forward, 0.1);
        turn = Deadband(turn, 0.2);


        double v = forward * VMAX;
        double w = turn * WMAX * m_turnAdjustFactor;


        double lwheel = v + (w * HALF_TRACKWIDTH);
        double rwheel = v - (w * HALF_TRACKWIDTH);

        double k;
        k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
        lwheel *= k;
        rwheel *= k;

        m_gearboxGauche.setSpeed(lwheel, 0);
        m_gearboxGauche.setSpeed(lwheel, 1);
        m_gearboxDroite.setSpeed(rwheel, 0);
        m_gearboxDroite.setSpeed(rwheel, 1);
    }
    else
    {
        characterization.setSpeedDriveOld(&m_gearboxGauche, &m_gearboxDroite);
    }
}

void Robot::DriveA(double forward, double turn)
{
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.1);
    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    m_targetLeftSpeed = lwheel * VMAX;
    m_targetRightSpeed = rwheel * VMAX;
}
void Robot::DriveB()
{
    m_joystick.updateVelocityAndAcceleration(&m_va_left, &m_va_max, m_targetLeftSpeed, 0.005);
    m_joystick.updateVelocityAndAcceleration(&m_va_right, &m_va_max, m_targetRightSpeed, 0.005);

    m_gearboxDroite.setVoltage(units::volt_t(m_gearboxDroite.getKv().getVoltage(0, &m_va_left)), 0);
    // m_gearboxDroite.setVoltage(units::volt_t(m_gearboxDroite.getKv().getVoltage(1, &m_va_left)), 1);
    m_gearboxGauche.setVoltage(units::volt_t(m_gearboxGauche.getKv().getVoltage(2, &m_va_left)), 0);
    // m_gearboxGauche.setVoltage(units::volt_t(m_gearboxGauche.getKv().getVoltage(3, &m_va_left)), 1);
}


#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<Robot>();
}

#endif
