// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "subsystems/Gearbox.h"
#include "lib/CSVLogFile.h"
#include "lib/Utils.h"
#include "lib/CustomMaths.h"
#include "subsystems/Tests.h"
#include "subsystems/Joystick.h"


#define rightInverte true
#define leftInverte false
#define leftMotor 2
#define leftMotorFollower 3
#define rightMotor 1
#define rightMotorFollower 4
#define leftEncoderChannelA 2
#define leftEncoderChannelB 3
#define rightEncoderChannelA 0
#define rightEncoderChannelB 1




class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  
  // void DriveOld(double forward, double turn);
  void Drive(double jy, double jx);


 private:
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;

  CSVLogFile *m_LogFile, *m_LogFileDriving;

  Gearbox m_gearboxGauche{leftMotor, leftMotorFollower, leftEncoderChannelA, leftEncoderChannelB, leftInverte};
  Gearbox m_gearboxDroite{rightMotor, rightMotorFollower, rightEncoderChannelA, rightEncoderChannelB, rightInverte};

  nt::NetworkTableEntry m_LogFilename, m_PowerEntry, m_logGyro, m_LogFilenameDriving;

  frc::ADXRS450_Gyro m_gyro{frc::SPI::Port::kOnboardCS0};//gyro definition

  Joystick m_joystick;

  bool m_override = false;
  bool m_isLogging = false;
  double m_ramp = 0;
  double m_time0;

  Tests tests;
};
