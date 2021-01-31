#pragma once

#include "subsystems/Gearbox.h"



// void Gearbox::setMotorCoefficients(unsigned int motorID, unsigned int isBackward, double kv, double ka, double vintersept)
// {
//     k_lut[motorID][isBackward][0] = kv;
//     k_lut[motorID][isBackward][1] = ka;
//     k_lut[motorID][isBackward][2] = vintersept;
// }

void Gearbox::setMotorCoefficients(uint isBackward)
{

    if(m_isLeft){
        //Left 1 Forward
        m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.80842, 0.16071, 0.1467);
        //Left 2 Forward
        m_kv.setMotorCoefficients(m_moteur2ID, isBackward, 2.80833, 0.1549, 0.14571);
        //Left 1 Backward
        m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.80397, 0.14271, -0.15557);
        //Left 2 Backward
        m_kv.setMotorCoefficients(m_moteur2ID, !isBackward, 2.80367, 0.13479, -0.15524);

    }else{
        //Right 1 Forward
        m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.80423, 0.13559, 0.16904);
        //Right 2 Forward
        m_kv.setMotorCoefficients(m_moteur2ID, isBackward, 2.80524, 0.13376, 0.16744);
        //Right 1 Backward
        m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.83487, 0.12593, -0.14335);
        //Right 2 Backward
        m_kv.setMotorCoefficients(m_moteur2ID, !isBackward, 2.83517, 0.12205, -0.14276);
    }
}



double Gearbox::getVoltage(unsigned int motorID, const VA *pva)
{
    int isBackward = (pva->m_speed < 0) ? 1 : 0;
    if (pva->m_speed == 0)
        return 0;
    return k_lut[motorID][isBackward][0] * pva->m_speed + k_lut[motorID][isBackward][1] * pva->m_acceleration + k_lut[motorID][isBackward][2];
}

Gearbox::Gearbox(int moteur1ID, int moteur2ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft){
    
    // Motors and encoders of the gearbox declaration
    rev::CANSparkMax m_moteur1{moteur1ID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_moteur2{moteur2ID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder m_encodeur1{m_moteur1};
    rev::CANEncoder m_encodeur2{m_moteur2};
    frc::Encoder m_encodeurExterne{encodeurChannelA, encodeurChannelB, isInverted, frc::Encoder::k2X};

    m_isLeft = isLeft;
    m_moteur1ID = moteur1ID;
    m_moteur2ID = moteur2ID;

}

void Gearbox::resetExternalEncodeur(){
    m_encodeurExterne.Reset();
}

void Gearbox::disableVoltageCompensation(){
    m_moteur1.DisableVoltageCompensation();
    m_moteur2.DisableVoltageCompensation();
}

void Gearbox::setIdleMode(rev::CANSparkMax::IdleMode mode){
    m_moteur1.SetIdleMode{mode};
    m_moteur2.SetIdleMode{mode};
}

void Gearbox::setInverted(bool invertion){
    m_moteur2.setInverted(invertion);
    m_moteur1.setInverted(invertion);
}

void Gearbox::setSpeed(VA *va){
    m_moteur1.set(m_kv.getVoltage(m_moteur1ID, &va) / m_moteur1.GetBusVoltage());
    m_moteur2.Set(m_kv.getVoltage(m_moteur2ID, &va) / m_moteur2.GetBusVoltage());
}


//kinetic to voltage
void KineticToVoltage::setMotorCoefficients(uint motorID, uint isBackward, double kv, double ka, double vintersept)
{
    k_lut[motorID][isBackward][0] = kv;
    k_lut[motorID][isBackward][1] = ka;
    k_lut[motorID][isBackward][2] = vintersept;
}

double KineticToVoltage::getVoltage(uint motorID, const VA *pva)
{
    int isBackward = (pva->m_speed < 0) ? 1 : 0;
    if (pva->m_speed == 0)
        return 0;
    return k_lut[motorID][isBackward][0] * pva->m_speed + k_lut[motorID][isBackward][1] * pva->m_acceleration + k_lut[motorID][isBackward][2];
}