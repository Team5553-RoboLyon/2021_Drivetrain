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
};

class KineticToVoltage
{

  //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
  double k_lut[4][2][3];

public:
  //void SetMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept);
  void setMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept);
  double getVoltage(uint motorID, const VA *pva);
};

class Gearbox
{

  public:
    void resetExternalEncodeur();
    void disableVoltageCompensation();
    // void setIdleMode(rev::CANSparkMax::IdleMode mode);
    void setIdleMode(string mode);
    void setInverted(bool invertion);
    Gearbox();
    // Gearbox(int moteur0ID, int moteur1ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft);
    void setMotorCoefficients(uint isBackward);
    void setSpeed(VA va);
    double getBusVoltage(int moteurID);
    double getAppliedOutput(int moteurID);
    double getExternalEncoderDistance();
    double getInternalEncoderPosition(int moteurID);
    double getOutputCurrent(int moteurID);
    void externalEncoderReset();

  private:

      //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
      double k_lut[2][2][3];
      double getVoltage(unsigned int motorID, const VA *pva);
      // void setMotorCoefficients(unsigned int motorID, unsigned int isBackward, double kv, double ka, double vintersept);

      bool m_isLeft;

      int m_moteur0ID;
      int m_moteur1ID;

      KineticToVoltage m_kv;
      rev::CANSparkMax m_moteur0;
      rev::CANSparkMax m_moteur1;
      rev::CANEncoder m_encodeur0;
      rev::CANEncoder m_encodeur1;
      frc::Encoder m_encodeurExterne;

  };


