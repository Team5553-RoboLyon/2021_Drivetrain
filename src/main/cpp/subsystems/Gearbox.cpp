#pragma once

#include "subsystems/Gearbox.h"



 Gearbox::Gearbox(int moteur0ID, int moteur1ID, unsigned int encodeurChannelA, unsigned int encodeurChannelB, bool isInverted, bool isLeft)
 {
    
    // Motors and encoders of the {gearbox declaration
    m_moteur0.~CANSparkMax();
    new(&m_moteur0) rev::CANSparkMax{moteur0ID, rev::CANSparkMax::MotorType::kBrushless};
    m_moteur1.~CANSparkMax();
    new(&m_moteur1) rev::CANSparkMax{moteur1ID, rev::CANSparkMax::MotorType::kBrushless};

    m_encodeur0.~CANEncoder();
    new(&m_encodeur0) rev::CANEncoder{m_moteur0.GetEncoder()};
    m_encodeur1.~CANEncoder();
    new(&m_encodeur1) rev::CANEncoder{m_moteur1.GetEncoder()};

    m_encodeurExterne.~Encoder();
    new(&m_encodeurExterne) frc::Encoder{encodeurChannelA, encodeurChannelB, isInverted, frc::Encoder::k4X};

    m_isLeft = isLeft;
    m_moteur0ID = moteur0ID;
    m_moteur1ID = moteur1ID;

 }


void Gearbox::setMotorCoefficients(uint isBackward)
{
    if(m_isLeft){
    //Left 1 Forward
    m_kv.setMotorCoefficients(m_moteur0ID, isBackward, 2.815697532731544, 0.4694670372587819, 0.11096684006625335);
    //Left 2 Forward
    m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.8171477288641165, 0.478564016576128, 0.10975114086308402);
    //Left 1 Backward
    m_kv.setMotorCoefficients(m_moteur0ID, !isBackward, 2.8526153046254596, 0.4060315027282673, -0.08529919470860658);
    //Left 2 Backward
    m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.853672567242419, 0.40031111541410647, -0.08431872256237849);

    }else{
    //Right 1 Forward
    m_kv.setMotorCoefficients(m_moteur0ID, isBackward, 2.8024631236903135, 0.4403635472698452, 0.10741975250277491);
    //Right 2 Forward
    m_kv.setMotorCoefficients(m_moteur1ID, isBackward, 2.801981945589249, 0.43687098372033967, 0.10747542821680156);
    //Right 1 Backward
    m_kv.setMotorCoefficients(m_moteur0ID, !isBackward, 2.7974216902473548, 0.38833514223084226, -0.10863199131460277);
    //Right 2 Backward
    m_kv.setMotorCoefficients(m_moteur1ID, !isBackward, 2.7967290044994533, 0.38680645837677974, -0.10898343976134406);
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

void Gearbox::setIdleMode(rev::CANSparkMax::IdleMode mode){
        m_moteur0.SetIdleMode(mode);
        m_moteur1.SetIdleMode(mode);
}

void Gearbox::setInverted(bool invertion){
    m_moteur1.SetInverted(invertion);
    m_moteur0.SetInverted(invertion);
}

// void Gearbox::setSpeed(VA va){
//     m_moteur0.Set(m_kv.getVoltage(m_moteur0ID, &va) / m_moteur0.GetBusVoltage());
//     m_moteur1.Set(m_kv.getVoltage(m_moteur1ID, &va) / m_moteur1.GetBusVoltage());
// }
KineticToVoltage Gearbox::getKv(){
    return m_kv;
}

void Gearbox::setSpeed(double speed, bool moteur){
    if (!moteur){
        m_moteur0.Set(speed);
    }
    if(moteur){
       m_moteur1.Set(speed); 
    }
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

void Gearbox::restoreFactoryDefaults(){
    m_moteur0.RestoreFactoryDefaults();
    m_moteur1.RestoreFactoryDefaults();
}
void Gearbox::setOpenLoopRampRate(float time_ramp){
    m_moteur0.SetOpenLoopRampRate(time_ramp);
    m_moteur1.SetOpenLoopRampRate(time_ramp);

}

void Gearbox::setDistancePerPulse(int pulse){
    m_encodeurExterne.SetDistancePerPulse(pulse);
}

void Gearbox::setSamplesToAverage(int average){
    m_encodeurExterne.SetSamplesToAverage(average);
}

bool Gearbox::getInverted(bool moteur){
if(!moteur){
    return m_moteur0.GetInverted();
}
if (moteur){
    return m_moteur1.GetInverted();
}
}

int Gearbox::getExternalEncoder(){
    return m_encodeurExterne.Get();
}

int Gearbox::getExternalEncoderRaw(){
    return m_encodeurExterne.GetRaw();
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