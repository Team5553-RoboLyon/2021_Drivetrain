#pragma once

#include "subsystems/Gearbox.h"


// void Gearbox::setMotorCoefficients(unsigned int motorID, unsigned int isBackward, double kv, double ka, double vintersept)
// {
//     k_lut[motorID][isBackward][0] = kv;
//     k_lut[motorID][isBackward][1] = ka;
//     k_lut[motorID][isBackward][2] = vintersept;
// }

// Gearbox::Gearbox(int moteur0ID, int moteur1ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft)
// {
    
//     // Motors and encoders of the gearbox declaration
//     rev::CANSparkMax m_moteur0{moteur0ID, rev::CANSparkMax::MotorType::kBrushless};
//     rev::CANSparkMax m_moteur1{moteur1ID, rev::CANSparkMax::MotorType::kBrushless};
//     rev::CANEncoder m_encodeur0{m_moteur0.GetEncoder()};
//     rev::CANEncoder m_encodeur1{m_moteur1.GetEncoder()};
//     frc::Encoder m_encodeurExterne{encodeurChannelA, encodeurChannelB, isInverted, frc::Encoder::k2X};

//     m_isLeft = isLeft;
//     m_moteur0ID = moteur0ID;
//     m_moteur1ID = moteur1ID;

// }

Gearbox::Gearbox()
{

}

void Gearbox::setMotorCoefficients(uint isBackward)
{

    if(m_isLeft){
        //Left 1 Forward
        m_kv.setMotorCoefficients(m_moteur0ID, isBackward, 2.80842, 0.16071, 0.1467);
        //Left 2 Forward
        m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.80833, 0.1549, 0.14571);
        //Left 1 Backward
        m_kv.setMotorCoefficients(m_moteur0ID, !isBackward, 2.80397, 0.14271, -0.15557);
        //Left 2 Backward
        m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.80367, 0.13479, -0.15524);

    }else{
        //Right 1 Forward
        m_kv.setMotorCoefficients(m_moteur0ID, isBackward, 2.80423, 0.13559, 0.16904);
        //Right 2 Forward
        m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.80524, 0.13376, 0.16744);
        //Right 1 Backward
        m_kv.setMotorCoefficients(m_moteur0ID, !isBackward, 2.83487, 0.12593, -0.14335);
        //Right 2 Backward
        m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.83517, 0.12205, -0.14276);
    }
}



double Gearbox::getVoltage(unsigned int motorID, const VA *pva)
{
    int isBackward = (pva->m_speed < 0) ? 1 : 0;
    if (pva->m_speed == 0){
        return 0;
    }
    return k_lut[motorID][isBackward][0] * pva->m_speed + k_lut[motorID][isBackward][1] * pva->m_acceleration + k_lut[motorID][isBackward][2];
}


void Gearbox::resetExternalEncodeur(){
    m_encodeurExterne.Reset();
}

void Gearbox::disableVoltageCompensation(){
    m_moteur0.DisableVoltageCompensation();
    m_moteur1.DisableVoltageCompensation();
}

void Gearbox::setIdleMode(string mode){
    assert (mode == "kBrake" || mode == "kCoast");
    if(mode == "kBrake"){
        m_moteur0.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_moteur1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    if(mode == "kBrake"){
        m_moteur0.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
        m_moteur1.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

void Gearbox::setInverted(bool invertion){
    m_moteur1.SetInverted(invertion);
    m_moteur0.SetInverted(invertion);
}

void Gearbox::setSpeed(VA va){
    m_moteur0.Set(m_kv.getVoltage(m_moteur0ID, &va) / m_moteur0.GetBusVoltage());
    m_moteur1.Set(m_kv.getVoltage(m_moteur1ID, &va) / m_moteur1.GetBusVoltage());
}

double Gearbox::getBusVoltage(int moteurID){
    assert (0 <= moteurID);
    assert (moteurID >= 1);
        if(moteurID == 0){
        return m_moteur0.GetBusVoltage();
    }
    if (moteurID == 1){
        return m_moteur1.GetBusVoltage();
    }
}

double Gearbox::getAppliedOutput(int moteurID){
    assert (0 <= moteurID);
    assert (moteurID >= 1);
    if(moteurID == 0){
        return m_moteur0.GetAppliedOutput();
    }
    if (moteurID == 1){
        return m_moteur1.GetAppliedOutput();
    }
}

double Gearbox::getExternalEncoderDistance(){
    return  m_encodeurExterne.GetDistance();
}

double Gearbox::getInternalEncoderPosition(int moteurID){
    assert (0 <= moteurID);
    assert (moteurID >= 1);
    if(moteurID == 0){
        return m_encodeur0.GetPosition();
    }
    if (moteurID == 1){
        return m_encodeur1.GetPosition();
    }
}

double Gearbox::getOutputCurrent(int moteurID){
    assert (0 <= moteurID);
    assert (moteurID >= 1);
    if(moteurID == 0){
        return m_moteur0.GetOutputCurrent();
    }
    if (moteurID == 1){
        return m_moteur1.GetOutputCurrent();
    }
}

void Gearbox::externalEncoderReset(){
    m_encodeurExterne.Reset();
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