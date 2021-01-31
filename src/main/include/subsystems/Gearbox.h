#pragma once

#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>

typedef struct VA VA;
struct VA
{
    double m_speed;
    double m_acceleration;
};

class Gearbox
{

  public:
    void resetExternalEncodeur();
    void disableVoltageCompensation();
    void setIdleMode(rev::CANSparkMax::IdleMode mode);
    void setInverted(bool invertion);
    Gearbox(int moteur1ID, int moteur2ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft);
    void setMotorCoefficients(uint isBackward);
    void setSpeed(VA *va);

  private:

      //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
      double k_lut[2][2][3];
      double getVoltage(unsigned int motorID, const VA *pva);
      // void setMotorCoefficients(unsigned int motorID, unsigned int isBackward, double kv, double ka, double vintersept);

      bool m_isLeft;

      int m_moteur1ID;
      int m_moteur2ID;

      KineticToVoltage m_kv;
      rev::CANSparkMax m_moteur1;
      rev::CANSparkMax m_moteur2;
      rev::CANEncoder m_encodeur1;
      rev::CANEncoder m_encodeur2;
      frc::Encoder m_encodeurExterne;

  };


class KineticToVoltage
{

  //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
  double k_lut[4][2][3];

public:
  void SetMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept);
  double getVoltage(uint motorID, const VA *pva);
};