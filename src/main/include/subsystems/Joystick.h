// #if XBOX_CONTROLLER
#include <frc/XboxController.h>
// #else
#include <frc/Joystick.h>
// #endif
#include <iostream>

#include "lib/CustomMaths.h"
#include "lib/Characterization.h"

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX (((2.0 * VMAX) / TRACKWIDTH) / 1.7) // vitesse angulaire Max theorique	.. à modifier avec Garice

class Joystick {

public: 
    void getSpeedsAndAccelerations(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy);
    void getSpeedsAndAccelerationsNew(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy);
    void updateVelocityAndAcceleration(VA *pva, const VA *pva_max, const double target_speed, const double dt);
    double getX(bool isRight);
    double getY(bool isRight);
    double getZ(bool isRight);

    bool getXButtonPressed();
    bool getBButtonPressed();
    bool getAButtonPressed();
    bool getYButtonPressed();


    double getThrottle(bool isRight);

private:

  frc::XboxController m_driverController{0};

  frc::Joystick m_leftHandController{0};
  frc::Joystick m_rightHandController{1};
  double m_turnAdjustFactor = 0.6;

};