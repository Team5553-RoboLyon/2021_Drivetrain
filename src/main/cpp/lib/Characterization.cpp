#include "lib/Characterization.h"

void Characterization::initializeTestData() {
    int i;
    // Low Voltages
    for (i = 0; i < TEST_LOWVOLTAGE_NB; i++) {
        TestData[i * 2].m_voltage = TEST_LOWVOLTAGE_MAX * (double) (i + 1) / (double) TEST_LOWVOLTAGE_NB;
        TestData[i * 2].m_flags = 0;

        TestData[i * 2 + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX * (double) (i + 1) / (double) TEST_LOWVOLTAGE_NB);
        TestData[i * 2 + 1].m_flags = 0;
    }
    // Medium Voltages
    for (i = 0; i < TEST_MEDIUMVOLTAGE_NB; i++) {
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_voltage = TEST_LOWVOLTAGE_MAX +
                                                           (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) *
                                                           (double) (i + 1) / (double) TEST_MEDIUMVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_voltage = -(TEST_LOWVOLTAGE_MAX +
                                                                 (TEST_MEDIUMVOLTAGE_MAX - TEST_LOWVOLTAGE_MAX) *
                                                                 (double) (i + 1) / (double) TEST_MEDIUMVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    // High Voltages
    for (i = 0; i < TEST_HIGHVOLTAGE_NB; i++) {
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_voltage = TEST_MEDIUMVOLTAGE_MAX +
                                                                                   (TEST_HIGHVOLTAGE_MAX -
                                                                                    TEST_MEDIUMVOLTAGE_MAX) *
                                                                                   (double) (i + 1) /
                                                                                   (double) TEST_HIGHVOLTAGE_NB;
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i)].m_flags = 0;

        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_voltage = -(TEST_MEDIUMVOLTAGE_MAX +
                                                                                         (TEST_HIGHVOLTAGE_MAX -
                                                                                          TEST_MEDIUMVOLTAGE_MAX) *
                                                                                         (double) (i + 1) /
                                                                                         (double) TEST_HIGHVOLTAGE_NB);
        TestData[2 * (TEST_LOWVOLTAGE_NB + TEST_MEDIUMVOLTAGE_NB + i) + 1].m_flags = 0;
    }

    Message = frc::Shuffleboard::GetTab("voltage").Add("Message", "").WithWidget(
            frc::BuiltInWidgets::kTextView).GetEntry();
}

void Characterization::waitingMessageTests() {
    char txt[256];
    if (ISFLAG_OFF(TestData[CurrentTestID].m_flags, FLAG_TestSpecs_Done)) {
        sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En Attente ... Appuyer sur A pour Démarrer.", CurrentTestID,
                TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    } else {
        sprintf(txt,
                "TEST %d / %d [ %.2f Volts ]. DEJA EFFECTUE ! ... Appuyer sur A pour Démarrer à nouveau.( Le précédent LOGFILE sera conservé.)",
                CurrentTestID, TEST_TOTAL_NB, TestData[CurrentTestID].m_voltage);
        Message.SetString(txt);
    }
}

void Characterization::allFinishedMessageTests() {
    char txt[256];
    sprintf(txt, "%d / %d  TESTS EFFECTUES ! ", TEST_TOTAL_NB * 2, TEST_TOTAL_NB * 2);
    Message.SetString(txt);
}

void Characterization::inProgressMessageTests() {
    char txt[256];
    sprintf(txt, "TEST %d / %d [ %.2f Volts ]. En cours ... Appuyer à nouveau sur A pour arrêter.", CurrentTestID,
            TEST_TOTAL_NB * 2, TestData[CurrentTestID].m_voltage);
    Message.SetString(txt);
}