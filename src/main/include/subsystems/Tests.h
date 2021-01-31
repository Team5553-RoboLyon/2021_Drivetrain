#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include "lib/Utils.h"

//voltage value definition
#define TEST_TOTAL_NB (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + TEST_HIGHVOLTAGE_NB)
#define TEST_MEDIUMVOLTAGE_NB 5    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_LOWVOLTAGE_MAX,TEST_MEDIUMVOLTAGE_MAX] volts	... 20 ou 25 ?
#define TEST_MEDIUMVOLTAGE_MAX 1.0 // Volts
#define TEST_HIGHVOLTAGE_NB 44    // Nombre de tests ( subdivisions ) sur l'intervalle ]TEST_MEDIUMVOLTAGE_MAX,TEST_HIGHVOLTAGE_MAX] volts... 12 ou 24 ?
#define TEST_HIGHVOLTAGE_MAX 12.0 // Volts
#define TEST_LOWVOLTAGE_NB 10    // Nombre de tests ( subdivisions ) sur l'intervalle ]0,TEST_LOWVOLTAGE_MAX] volts						... 10 ou 20 ?
#define TEST_LOWVOLTAGE_MAX 0.15 // Volts

//flag definition
#define FLAG_TestSpecs_Done 1
#define ISFLAG_OFF(val, flag) (!((val) & (flag))) 




typedef struct TestSpecs TestSpecs;
struct TestSpecs
{
    unsigned long m_flags;
    double m_voltage;
};







class Tests{

    public:
        void initializeTestData();
        void messageTestEnAttente();
        void messageTestTousEffectues();
        void messageTestEnCours(); 


    private:
        TestSpecs TestData[TEST_TOTAL_NB * 2];
        int CurrentTestID = 0;
        nt::NetworkTableEntry Message;


};