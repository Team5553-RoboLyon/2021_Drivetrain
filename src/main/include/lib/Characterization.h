#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>

#define TRACKWIDTH 0.61f
#define HALF_TRACKWIDTH (TRACKWIDTH / 2.0f)

#define AMAX 5.1 // Acceleration Max  au PIF .. à définir aux encodeurs
#define VMAX 3.4 // vitesse Max  théorique (3,395472 sur JVN-DT) .. à vérifier aux encodeurs
#define WMAX                       \
    (((2.0 * VMAX) / TRACKWIDTH) / \
     1.7) // vitesse angulaire Max theorique	.. à modifier avec Garice

// Flags Manipulation
#define FLAG_TOGGLE(val, flag) ((val) ^= (flag))
#define FLAG_ON(val, flag) ((val) |= (flag))
#define FLAG_OFF(val, flag) ((val) &= ~(flag))
#define ISFLAG_ON(val, flag) ((val) & (flag))                                   // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define ISFLAG_OFF(val, flag) (!((val) & (flag)))                               // !! ZERO or NON ZERO value !!! BE AWARE THAT NON ZERO DOESN'T MEAN 1 !!!
#define FLAGS_TEST(val, bmask, flags) (((val) & (bmask)) == (flags))            // NFALSE or NTRUE
#define SET_FLAGS(val, bmask, flags) ((val) = (((val) & (~(bmask))) | (flags))) // RESET FLAGS BITS and Set them all like flags.
#define RESET_FLAGS(val, bmask)		((val)&=~(bmask)))						// Set all FLAGS BITS to ZERO

#define VOLTAGE_COMPENSATION_VALUE 11.5
// TEST *********************************************
#define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
#define TEST_LOWVOLTAGE_MAX 0.15 // Volts

#define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
#define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts

#define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
#define TEST_HIGHVOLTAGE_MAX 12.0 // Volts

#define TEST_TOTAL_NB (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + TEST_HIGHVOLTAGE_NB)

#define FLAG_TestSpecs_Done 1

#define TIME_RAMP 0

class Characterization
{
public:
    void initializeTestData();
    void waitingMessageTests();
    void allFinishedMessageTests();
    void inProgressMessageTests();

private:
    typedef struct TestSpecs TestSpecs;

    TestSpecs TestData[TEST_TOTAL_NB * 2];
    int CurrentTestID = 0;

    nt::NetworkTableEntry Message;
};

struct TestSpecs
{
    unsigned long m_flags;
    double m_voltage;
};