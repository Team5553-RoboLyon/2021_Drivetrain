#pragma once

#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
#include <assert.h>
#include <iostream>
#include <string>

typedef struct VA VA;
struct VA
{
  double m_speed;
  double m_acceleration;
  double m_jerk;
};

class KineticToVoltage
{

  double k_lut[4][2][3];

public:
  void setMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept);
  double getVoltage(uint motorID, const VA *pva);
};

class Gearbox
{

  public:
    void resetExternalEncodeur();
    void disableVoltageCompensation();
    void enableVoltageCompensation(double nominalVoltage);
    void setPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame frame, int periodMs, bool moteurID);
    void setIdleMode(rev::CANSparkMax::IdleMode mode);
    void setInverted(bool invertion);
    Gearbox(int moteur0ID, int moteur1ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft);
    void setMotorCoefficients(uint isBackward);
    // void setSpeed(VA va);
    void setSpeed(double speed, bool moteur);
    double getBusVoltage(int moteurID);
    double getAppliedOutput(int moteurID);
    double getExternalEncoderDistance();
    double getInternalEncoderPosition(int moteurID);
    double getOutputCurrent(int moteurID);
    void externalEncoderReset();
    void restoreFactoryDefaults();
    void setOpenLoopRampRate(float time_ramp);
    void setDistancePerPulse(int pulse);
    void setSamplesToAverage(int average);
    bool getInverted(bool moteur);
    int getExternalEncoder();
    int getExternalEncoderRaw();
    KineticToVoltage getKv();
    void setVoltage(units::voltage::volt_t output, bool moteurID);
    uint16_t getFaults(bool moteurID);



  private:

      double k_lut[2][2][3];
      double getVoltage(unsigned int motorID, const VA *pva);

      bool m_isLeft;

      int m_moteur0ID;
      int m_moteur1ID;

      KineticToVoltage m_kv;
      rev::CANSparkMax m_moteur0{0, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANSparkMax m_moteur1{1, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANEncoder m_encodeur0{m_moteur0.GetEncoder()};
      rev::CANEncoder m_encodeur1{m_moteur1.GetEncoder()};
      frc::Encoder m_encodeurExterne{0, 1, false, frc::Encoder::k4X};

  };


