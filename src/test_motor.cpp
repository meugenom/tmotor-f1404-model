/**
 * Testen des Motor-Modells für T-Motor F1404 KV4600
 * Verwendet die motor_model.m-Polynomkoeffizienten und die LUT-Daten aus motor_lut.h.
 * Ziel: Überprüfen der Genauigkeit der getMotorThrustNewtons()-Funktion gegen die Standtest-Daten des Datenblatts.
 */


#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "motor.h"
#include "motor_lut.h"

// ANSI Farben für bessere Lesbarkeit der Testergebnisse
#define COL_GREEN "\033[32m"
#define COL_RED   "\033[31m"
#define COL_RESET "\033[0m"

// Toleranz ±5% gegenüber den tatsächlichen Daten des Datenblatts
static constexpr float TOLERANCE = 0.05f;

bool approxEqual(float actual, float expected, float tol) {
    if (expected == 0.0f) return fabs(actual) < tol;
    return fabs(actual - expected) / fabs(expected) < tol;
}

// Ersetzen assert() — druckt das Ergebnis und beendet das Programm bei einem Fehler
static void check(bool ok, const char* expr, const char* file, int line) {
    if (!ok) {
        printf(COL_RED "✗ FAIL: %s\n  → %s:%d" COL_RESET "\n", expr, file, line);
        exit(1);
    }
}

#define CHECK(expr) check((expr), #expr, __FILE__, __LINE__)

void test_thrust_at_50_percent() {
    // Datasheet: 50% throttle, 15.93V → 184.21g = 1.807N
    float thrust = getMotorThrustNewtons(0.5f, 15.93f);
    float expected = 184.21f / 1000.0f * 9.81f; // 1.807N
    CHECK(approxEqual(thrust, expected, TOLERANCE));
    printf(COL_GREEN "✓" COL_RESET " thrust at 50%%: %.3fN (expected %.3fN)\n", thrust, expected);
}

void test_thrust_at_75_percent() {
    // Datasheet: 75% throttle, 15.81V → 287.60g = 2.821N
    float thrust = getMotorThrustNewtons(0.75f, 15.81f);
    float expected = 287.60f / 1000.0f * 9.81f;
    CHECK(approxEqual(thrust, expected, TOLERANCE));
    printf(COL_GREEN "✓" COL_RESET " thrust at 75%%: %.3fN (expected %.3fN)\n", thrust, expected);
}


void test_thrust_at_100_percent() {
    // Datasheet: 100% throttle, 15.64V → 344.73g = 3.381N
    float thrust = getMotorThrustNewtons(1.0f, 15.64f);
    float expected = 344.73f / 1000.0f * 9.81f;
    CHECK(approxEqual(thrust, expected, TOLERANCE));
    printf(COL_GREEN "✓" COL_RESET " thrust at 100%%: %.3fN (expected %.3fN)\n", thrust, expected);
}

void test_zero_throttle() {
    float thrust = getMotorThrustNewtons(0.0f, 15.8f);
    CHECK(thrust == 0.0f);
    printf(COL_GREEN "✓" COL_RESET " zero throttle → zero thrust\n");
}

void test_thrust_monotonic() {
    // Thrust should increase with throttle — monotonicity
    float prev = 0.0f;
    for (float t = 0.1f; t <= 1.0f; t += 0.1f) {
        float thrust = getMotorThrustNewtons(t, 15.8f);
        CHECK(thrust >= prev);
        prev = thrust;
    }
    printf(COL_GREEN "✓" COL_RESET " thrust is monotonically increasing\n");
}

void test_voltage_effect() {
    // At lower voltage, thrust should be lower
    float thrust_full = getMotorThrustNewtons(0.75f, 16.8f);
    float thrust_low  = getMotorThrustNewtons(0.75f, 13.2f);
    CHECK(thrust_full > thrust_low);
    printf(COL_GREEN "✓" COL_RESET " lower voltage → lower thrust\n");
}

void test_hover_quad_250g() {
    // Drone 250g, 4 motors F1404 — should hover at ~17.7% throttle (4S ~15.8V)
    // Needed: 250g / 4 = 62.5g per motor
    // Hover throttle calculated in motor_model.m from polynomial
    static constexpr float HOVER_VOLTAGE  = 15.8f;  // slightly discharged 4S
    static constexpr float HOVER_THROTTLE = MOTOR_HOVER_THROTTLE_250G;
    static constexpr float DRONE_MASS_G   = 250.0f;
    static constexpr float MOTORS         = 4.0f;
    static constexpr float HOVER_MARGIN   = 0.10f;  // tolerance ±10% (extrapolation, not measurement)

    float thrust_per_motor_N = getMotorThrustNewtons(HOVER_THROTTLE, HOVER_VOLTAGE);
    float total_thrust_g     = thrust_per_motor_N * MOTORS / 9.81f * 1000.0f;

    // Total thrust should keep the drone within ±10%
    float expected_g = DRONE_MASS_G;
    CHECK(approxEqual(total_thrust_g, expected_g, HOVER_MARGIN));
    printf(COL_GREEN "✓" COL_RESET " hover 250g quad: throttle=%.1f%%, total=%.1fg (need %.0fg)\n",
           HOVER_THROTTLE * 100.0f, total_thrust_g, expected_g);
}


// Toleranz für Stromtests: 10% — Modell wurde primär auf Schub optimiert
static constexpr float TOLERANCE_CURRENT = 0.10f;

void test_current_at_50_percent() {
    // Datasheet: 50% throttle, 15.93V → 5.23A
    float current  = getMotorCurrentAmps(0.5f, 15.93f);
    float expected = 5.23f;
    CHECK(approxEqual(current, expected, TOLERANCE_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 50%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_at_75_percent() {
    // Datasheet: 75% throttle, 15.81V → 11.32A
    float current  = getMotorCurrentAmps(0.75f, 15.81f);
    float expected = 11.32f;
    CHECK(approxEqual(current, expected, TOLERANCE_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 75%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_at_100_percent() {
    // Datasheet: 100% throttle, 15.64V → 17.54A
    float current  = getMotorCurrentAmps(1.0f, 15.64f);
    float expected = 17.54f;
    CHECK(approxEqual(current, expected, TOLERANCE_CURRENT));
    printf(COL_GREEN "✓" COL_RESET " current at 100%%: %.2fA (expected %.2fA)\n", current, expected);
}

void test_current_monotonic() {
    // Strom steigt mit Gas monoton an
    float prev = 0.0f;
    for (float t = 0.1f; t <= 1.0f; t += 0.1f) {
        float current = getMotorCurrentAmps(t, 15.8f);
        CHECK(current >= prev);
        prev = current;
    }
    printf(COL_GREEN "✓" COL_RESET " current is monotonically increasing\n");
}

void test_current_voltage_quadratic() {
    // V²-Skalierung: I(V_high) / I(V_low) ≈ (V_high/V_low)²
    float v_high = 16.8f;
    float v_low  = 14.0f;
    float i_high = getMotorCurrentAmps(0.75f, v_high);
    float i_low  = getMotorCurrentAmps(0.75f, v_low);
    float ratio_actual   = i_high / i_low;
    float ratio_expected = (v_high / v_low) * (v_high / v_low);  // (16.8/14.0)² = 1.44
    CHECK(approxEqual(ratio_actual, ratio_expected, 0.05f));  // ±5%
    printf(COL_GREEN "✓" COL_RESET " current V²-scaling: I(%.1fV)/I(%.1fV) = %.3f (expected %.3f)\n",
           v_high, v_low, ratio_actual, ratio_expected);
}

int main() {
    printf("\n=== Motor Model Tests: F1404 KV4600 ===\n");

    test_zero_throttle();
    test_thrust_at_50_percent();
    test_thrust_at_75_percent();
    test_thrust_at_100_percent();
    test_thrust_monotonic();
    test_voltage_effect();
    test_hover_quad_250g();

    printf("\n--- Stromtests (I ∝ V²) ---\n");
    test_current_at_50_percent();
    test_current_at_75_percent();
    test_current_at_100_percent();
    test_current_monotonic();
    test_current_voltage_quadratic();

    printf(COL_GREEN "\n✓ All tests passed.\n" COL_RESET);
    float errs[] = {
        fabs(getMotorThrustNewtons(0.50f, 15.93f) - 184.21f/1000.0f*9.81f) / (184.21f/1000.0f*9.81f) * 100.0f,
        fabs(getMotorThrustNewtons(0.75f, 15.81f) - 287.60f/1000.0f*9.81f) / (287.60f/1000.0f*9.81f) * 100.0f,
        fabs(getMotorThrustNewtons(1.00f, 15.64f) - 344.73f/1000.0f*9.81f) / (344.73f/1000.0f*9.81f) * 100.0f,
    };
    float avg_err = (errs[0] + errs[1] + errs[2]) / 3.0f;
    printf("Model accuracy (50/75/100%%): %.1f%% / %.1f%% / %.1f%% | avg=%.1f%%\n", errs[0], errs[1], errs[2], avg_err);
    
    return 0;
}