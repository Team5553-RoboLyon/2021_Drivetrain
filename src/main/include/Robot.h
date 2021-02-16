// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include <string>


#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/shuffleboard/Shuffleboard.h>
#if IMU
#include <adi/ADIS16470_IMU.h>
#endif
#include <frc/LinearFilter.h>


#include "subsystems/Gearbox.h"
#include "lib/CSVLogFile.h"
#include "lib/Utils.h"
#include "lib/CustomMaths.h"
#include "subsystems/Joystick.h"
#include "lib/Characterization.h"
#include "Constant.h"

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

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX ((2.0 * VMAX) / TRACKWIDTH) // vitesse angulaire Max theorique	.. à modifier avec Garice

#define FLAG_ON(val, flag) ((val) |= (flag))




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
  
  void DriveOld(double forward, double turn);
  void Drive(double jy, double jx);


 private:
  VA m_va_left;
  VA m_va_right;
  VA m_va_max;

  // CSVLogFile *m_LogFile, *m_LogFileDriving;

  Gearbox m_gearboxGauche{leftMotor, leftMotorFollower, leftEncoderChannelA, leftEncoderChannelB, leftInverte, true};
  Gearbox m_gearboxDroite{rightMotor, rightMotorFollower, rightEncoderChannelA, rightEncoderChannelB, rightInverte, false};

  nt::NetworkTableEntry m_speedY, m_speedX, m_PowerEntry, m_logGyro, m_customEntry;

  frc::ADXRS450_Gyro m_gyro{frc::SPI::Port::kOnboardCS0};//gyro definition
  double init_x;
  double init_y;

  double m_time0;
  double m_ramp = 0;


  #if IMU
    frc::ADIS16470_IMU m_imu{};
    frc::LinearFilter<double> filterX = frc::LinearFilter<double>::MovingAverage(64);
    frc::LinearFilter<double> filterY = frc::LinearFilter<double>::MovingAverage(64);
  #endif

  Joystick m_joystick;

  bool m_override = false;
  bool m_isLogging = false;


  Characterization characterization;

  double m_turnAdjustFactor = 0.6;

  char m_invertedPrefix[8];
};
