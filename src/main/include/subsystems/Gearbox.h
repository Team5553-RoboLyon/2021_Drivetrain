#pragma once

typedef struct VA VA;
struct VA
{
    double m_speed;
    double m_acceleration;
};

class Gearbox
{
private:
    //k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
    double k_lut[2][2][3];
    double getVoltage(unsigned int motorID, const VA *pva);
    void SetMotorCoefficients(unsigned int motorID, unsigned int isBackward, double kv, double ka, double vintersept);
};