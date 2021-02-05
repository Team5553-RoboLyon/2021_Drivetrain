#include <frc/XboxController.h>
#include <iostream>

#include "lib/CustomMaths.h"
#include "lib/Characterization.h"


class Joystick {

public: 
    void getSpeedsAndAccelerations(VA *pva_left, VA *pva_right, const VA *pvamax, const double jx, const double jy);
    double getY();
    double getX();
    bool getXButtonPressed();
    bool getBButtonPressed();
    bool getAButtonPressed();
    bool getYButtonPressed();

private:
  frc::XboxController m_driverController{0};//create xbox controller
};