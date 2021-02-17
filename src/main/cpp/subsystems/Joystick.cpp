#include "subsystems/Joystick.h"


double Joystick::getSign(double number)
{
    if (number < 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}


double Joystick::getX(bool isRight){
    #if XBOX_CONTROLLER
        if(isRight){
            return m_driverController.GetX(frc::GenericHID::JoystickHand::kRightHand);
        }
        if(!isRight){
            return m_driverController.GetX(frc::GenericHID::JoystickHand::kLeftHand);
        }
    #else
        if(isRight){
            return m_rightHandController.GetX();
        }
        if(!isRight){
            return m_leftHandController.GetX();
        }
    #endif

}

double Joystick::getY(bool isRight){
    #if XBOX_CONTROLLER
        if(isRight){
            return m_driverController.GetY(frc::GenericHID::JoystickHand::kRightHand);
        }
        if(!isRight){
            return m_driverController.GetY(frc::GenericHID::JoystickHand::kLeftHand);
        }
    #else
        if(isRight){
            return m_rightHandController.GetY();
        }
        if(!isRight){
            return m_leftHandController.GetY();
        }
    #endif
}

double Joystick::getZ(bool isRight){
    #if XBOX_CONTROLLER
    if(isRight){
        return m_driverController.GetZ(frc::GenericHID::JoystickHand::kRightHand);
    }
    if(!isRight){
        return m_driverController.GetZ(frc::GenericHID::JoystickHand::kLeftHand);
    }
    #else
        if(isRight){
            return m_rightHandController.GetZ();
        }
        if(!isRight){
            return m_leftHandController.GetZ();
        }    
    #endif
}

void Joystick::getSpeedsAndAccelerations(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy)
{
    double premix_left;  //    (left) premixed output(-1.. + 1)
    double premix_right; //    (right)premixed output(-1.. + 1)

    double omega; //    pivot speed
    double blend;

    double blend_threshold = 0.75;

    if (jy >= 0)
    {
        // Forward
        premix_left = (jx >= 0.0) ? 1.0 : (1.0 + jx);
        premix_right = (jx >= 0.0) ? (1.0 - jx) : 1.0;
    }
    else
    {
        // Reverse
        premix_left = (jx >= 0.0) ? (1.0 - jx) : 1.0;
        premix_right = (jx >= 0.0) ? 1.0 : (1.0 + jx);
    }

    // Scale Drive output due to Joystick Y input (throttle)
    premix_left *= jy;
    premix_right *= jy;

    // Now calculate pivot amount
    // - Strength of pivot (nPivSpeed) based on Joystick X input
    // - Blending of pivot vs drive (blend) based on Joystick Y input
    omega = jx;
    blend = (NABS(jy) > blend_threshold) ? 0.0 : (1.0 - (NABS(jy) / blend_threshold));
    std::cout << blend << std::endl;
    double mix_left;
    double mix_right;

    // Calculate final mix of Drive and Pivot
    mix_left = (1.0 - blend) * premix_left + blend * (omega);
    mix_right = (1.0 - blend) * premix_right + blend * (-omega);

    double target_left_speed;
    double target_right_speed;

    target_left_speed = mix_left * pvamax->m_speed;
    target_right_speed = mix_right * pvamax->m_speed;

    double acc;
    double v_diff;

    //Left side
    acc = pvamax->m_acceleration * 0.02;
    v_diff = target_left_speed - pva_left->m_speed;

    if (v_diff < -acc)
    {
        pva_left->m_speed -= acc;
        pva_left->m_acceleration = pvamax->m_acceleration;
    }
    else if (v_diff > acc)
    {
        pva_left->m_speed += acc;
        pva_left->m_acceleration = pvamax->m_acceleration;
    }
    else
    {
        pva_left->m_speed = target_left_speed;
        pva_left->m_acceleration = 0;
    }

    //Right side
    acc = pvamax->m_acceleration * 0.02;
    v_diff = target_right_speed - pva_right->m_speed;

    if (v_diff < -acc)
    {
        pva_right->m_speed -= acc;
        pva_right->m_acceleration = pvamax->m_acceleration;
    }
    else if (v_diff > acc)
    {
        pva_right->m_speed += acc;
        pva_right->m_acceleration = pvamax->m_acceleration;
    }
    else
    {
        pva_right->m_speed = target_right_speed;
        pva_right->m_acceleration = 0;
    }
    std::cout << "vitesse : " << pva_right->m_speed << "   " << pva_left->m_speed << std::endl;
}

void Joystick::getSpeedsAndAccelerationsNew(VA *pva_left, VA *pva_right, const VA *pva_max, const double jx, const double jy)
{
    double target_left_speed;
    double target_right_speed;

    double v = jx * VMAX;
    double w = jy * WMAX;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    target_left_speed = lwheel * VMAX;
    target_right_speed = rwheel * VMAX;

    updateVelocityAndAcceleration(pva_left, pva_max, target_left_speed, 0.02);
    updateVelocityAndAcceleration(pva_right, pva_max, target_right_speed, 0.02);
}

void Joystick::updateVelocityAndAcceleration(VA *pva, const VA *pva_max, const double target_speed, const double dt)
{
    double dv0v1 = target_speed - pva->m_speed;
    double dv_a = getSign(pva->m_acceleration) * pva->m_acceleration * pva->m_acceleration / (2.0f * pva_max->m_jerk);
    double d_v = dv0v1 - dv_a;

    if (d_v < 0)
    {
        if (pva->m_acceleration <= -pva_max->m_acceleration)
        {
            pva->m_jerk = 0.0;
        }
        else
        {
            pva->m_jerk = -pva_max->m_jerk;
        }
    }
    else if (d_v > 0)
    {
        if (pva->m_acceleration >= pva_max->m_acceleration)
        {
            pva->m_jerk = 0.0;
        }
        else
        {
            pva->m_jerk = pva_max->m_jerk;
        }
    }
    else
    {
        pva->m_jerk = 0.0f;
    }

    double a = pva->m_acceleration + pva->m_jerk * dt;

    if (a > pva_max->m_acceleration)
    {
        a = pva_max->m_acceleration;
    }
    else if (a < -pva_max->m_acceleration)
    {
        a = -pva_max->m_acceleration;
    }

    double t = abs(a - pva->m_acceleration) / pva_max->m_jerk;

    double da = pva->m_acceleration * t + 0.5 * t * t * pva->m_jerk + a * (dt - t);
    if (getSign(dv0v1) != getSign(dv0v1 - da))
    {
        pva->m_acceleration = 0.0;
        pva->m_jerk = 0.0;
        pva->m_speed = target_speed;
    }
    else
    {
        pva->m_speed += da;
        pva->m_acceleration = a;
    }
}

double Joystick::getThrottle(bool isRight){
    #if XBOX_CONTROLLER
    #else
        if(isRight){
            return m_rightHandController.GetThrottle();
        }
        if(!isRight){
            return m_leftHandController.GetThrottle();
        }    
    #endif
}


bool Joystick::getXButtonPressed(){
    return m_driverController.GetXButtonPressed();
}

bool Joystick::getAButtonPressed(){
    return m_driverController.GetAButtonPressed();
}

bool Joystick::getYButtonPressed(){
    return m_driverController.GetYButtonPressed();
}
bool Joystick::getBButtonPressed(){
    return m_driverController.GetBButtonPressed();
}