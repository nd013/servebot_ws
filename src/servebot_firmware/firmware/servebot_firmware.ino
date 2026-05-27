/*
 * ESP32 Dual RMCS-2301 Motor Controller
 * + Quadrature Encoder Reading
 * + ROS2 / PS5 Teleop via UART2
 *
 * ── Pin Map ───────────────────────────────────────────────────────────────
 *
 *   ESP32 GPIO    RMCS-2301 / Encoder        Signal
 *   ──────────────────────────────────────────────────────────────────────
 *   GPIO 25 ──►  Driver #1  Pul+            Motor 1 LEFT  Step
 *   GPIO 26 ──►  Driver #1  Dir+            Motor 1 LEFT  Direction
 *   GPIO 18 ◄──  Motor 1 Encoder A          LEFT  enc A  (internal pull-up)
 *   GPIO 19 ◄──  Motor 1 Encoder B          LEFT  enc B  (internal pull-up)
 *
 *   GPIO 13 ──►  Driver #2  Pul+            Motor 2 RIGHT Step
 *   GPIO 14 ──►  Driver #2  Dir+            Motor 2 RIGHT Direction
 *   GPIO 32 ◄──  Motor 2 Encoder A          RIGHT enc A  (internal pull-up)
 *   GPIO 33 ◄──  Motor 2 Encoder B          RIGHT enc B  (internal pull-up)
 *
 *   GPIO 16 ◄──  Raspberry Pi 5 Pin 8  (TX) UART2 RX  (ROS2 bridge)
 *   GPIO 17 ──►  Raspberry Pi 5 Pin 10 (RX) UART2 TX
 *   GND     ───  Raspberry Pi 5 Pin 6  (GND)
 *
 *   ENA+ on both drivers  ──  External supply (wired externally, not ESP32)
 *
 * ── Wiring Notes ─────────────────────────────────────────────────────────
 *   1. All GNDs common: ESP32 GND, both driver GNDs, supply GND, RPi GND
 *   2. All encoder pins (18, 19, 32, 33) use internal INPUT_PULLUP —
 *      no external resistors needed.
 *   3. Keep encoder wires twisted pair, away from motor power cables.
 *   4. Set electronic gear ratio 1:1 on both RMCS-2301 drivers.
 *
 * ── ROS2 Serial Protocol (UART2, 115200 baud) ────────────────────────────
 *   RPi → ESP32:
 *     "J <ly> <rx>\n"    Joystick -100..+100  (from simple_serial_transmitter)
 *     "S\n"              Immediate hard stop
 *
 *   ESP32 → RPi:
 *     "READY\n"          On boot
 *     "OK STOPPED\n"     After S command or watchdog timeout
 *     "E <t1> <t2>\n"    Cumulative encoder ticks at 50 Hz
 *     "ERR ...\n"        Error messages
 *
 * ── Encoder Ticks ────────────────────────────────────────────────────────
 *   4x quadrature decoding (interrupt on CHANGE of both A and B channels).
 *   Ticks are cumulative signed counts sent to odometry_node at 50 Hz.
 *
 * ── Tank-Drive Mixing (J command) ────────────────────────────────────────
 *   L (left)  = (ly + rx) / 100   clamped ±1.0 → scaled to ±MAX_SPEED
 *   R (right) = (ly - rx) / 100   clamped ±1.0 → scaled to ±MAX_SPEED
 *
 * ── Watchdog ─────────────────────────────────────────────────────────────
 *   Motors stop 300 ms after last J command if no new command arrives.
 *
 * Requires: AccelStepper (Arduino Library Manager)
 */

#include <AccelStepper.h>
#include <HardwareSerial.h>

// ── Pin Definitions ────────────────────────────────────────────────────────

#define M1_STEP_PIN     25
#define M1_DIR_PIN      26

#define M2_STEP_PIN     13
#define M2_DIR_PIN      14

#define ENC1_A          18
#define ENC1_B          19
#define ENC2_A          32
#define ENC2_B          33

#define UART2_RX_PIN    16
#define UART2_TX_PIN    17

#define RPI_WAIT_TIMEOUT_MS     30000UL

// ── Robot & Drive Train Parameters ────────────────────────────────────────

#define WHEEL_RADIUS_M          0.060f                           // 60 mm
#define WHEEL_DIAMETER_MM       (WHEEL_RADIUS_M * 2.0f * 1000.0f)  // 120 mm
#define WHEEL_CIRCUMFERENCE_MM  (PI * WHEEL_DIAMETER_MM)            // 376.99 mm
#define WHEELBASE_MM            1480.0f

#define STEPS_PER_WHEEL_REV     46000L
#define STEPS_PER_MM            ((float)STEPS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM)

#define MAX_SPEED               3000.0f
#define DEFAULT_ACCEL           800.0f
#define MIN_PULSE_WIDTH_US      10
#define RUN_TARGET_STEPS        5000000L

#define WATCHDOG_MS             300
#define JOY_DEADZONE            0.05f

#define ENC_REPORT_HZ           50
#define ENC_REPORT_MS           (1000 / ENC_REPORT_HZ)

// ── AccelStepper Instances ─────────────────────────────────────────────────

AccelStepper motor1(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN);

// ── Encoder State ─────────────────────────────────────────────────────────

volatile long enc1_count = 0;
volatile long enc2_count = 0;

static const int8_t ENC_QEM[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

volatile uint8_t enc1_state = 0;
volatile uint8_t enc2_state = 0;

// ── Encoder ISRs ──────────────────────────────────────────────────────────

void IRAM_ATTR enc1_isr() {
    uint8_t ns = ((uint8_t)digitalRead(ENC1_A) << 1) | (uint8_t)digitalRead(ENC1_B);
    enc1_count += ENC_QEM[(enc1_state << 2) | ns];
    enc1_state = ns;
}

void IRAM_ATTR enc2_isr() {
    uint8_t ns = ((uint8_t)digitalRead(ENC2_A) << 1) | (uint8_t)digitalRead(ENC2_B);
    enc2_count += ENC_QEM[(enc2_state << 2) | ns];
    enc2_state = ns;
}

// ── Global State ──────────────────────────────────────────────────────────

String   serialBuf      = "";
String   serialBuf2     = "";
bool     throttleMode   = false;
uint32_t g_lastCmdMs    = 0;
bool     g_watchdogArmed = false;
uint32_t g_lastEncMs    = 0;

// ── Motion Helpers ─────────────────────────────────────────────────────────

void setSpeedMMperS(float mmps) {
    float sps = constrain(mmps * STEPS_PER_MM, 1.0f, MAX_SPEED);
    motor1.setMaxSpeed(sps);
    motor2.setMaxSpeed(sps);
}

void setAcceleration(float a) {
    motor1.setAcceleration(a);
    motor2.setAcceleration(a);
}

void waitForBoth() {
    while (motor1.isRunning() || motor2.isRunning()) {
        motor1.run();
        motor2.run();
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    delay(500);
}

void fullStop() {
    throttleMode    = false;
    g_watchdogArmed = false;
    motor1.stop();
    motor2.stop();
    while (motor1.isRunning() || motor2.isRunning()) {
        motor1.run();
        motor2.run();
    }
    motor1.setSpeed(0);
    motor2.setSpeed(0);
}

void resetDriverFault() {
    throttleMode = false;
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    motor1.setCurrentPosition(0);
    motor2.setCurrentPosition(0);
    Serial.println("   Step positions zeroed. Power-cycle 24 V to clear driver fault.");
}

// ── Motion Primitives ──────────────────────────────────────────────────────

void moveDistance(float mm) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    long steps = (long)(mm * STEPS_PER_MM);
    motor1.move(steps);
    motor2.move(steps);
    waitForBoth();
}

void rotateRobot(float angleDeg) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    float arcMM = (angleDeg / 360.0f) * PI * WHEELBASE_MM;
    long  steps  = (long)(arcMM * STEPS_PER_MM);
    motor1.move(-steps);
    motor2.move( steps);
    waitForBoth();
}

void rotWheelBoth(float deg) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    long steps = (long)((deg / 360.0f) * (float)STEPS_PER_WHEEL_REV);
    motor1.move(steps);
    motor2.move(steps);
    waitForBoth();
}

void rotWheelLeft(float deg) {
    motor1.setSpeed(0);
    long steps = (long)((deg / 360.0f) * (float)STEPS_PER_WHEEL_REV);
    motor1.move(steps);
    while (motor1.isRunning()) motor1.run();
    motor1.setSpeed(0);
    delay(500);
}

void rotWheelRight(float deg) {
    motor2.setSpeed(0);
    long steps = (long)((deg / 360.0f) * (float)STEPS_PER_WHEEL_REV);
    motor2.move(steps);
    while (motor2.isRunning()) motor2.run();
    motor2.setSpeed(0);
    delay(500);
}

// ── PS5 / ROS2 Joystick Control ───────────────────────────────────────────

static void applyJoystick(float ly, float rx) {
    float L = constrain(ly + rx, -100.0f, 100.0f) / 100.0f;
    float R = constrain(ly - rx, -100.0f, 100.0f) / 100.0f;

    if (fabsf(L) < JOY_DEADZONE) L = 0.0f;
    if (fabsf(R) < JOY_DEADZONE) R = 0.0f;

    if (L == 0.0f && R == 0.0f) {
        fullStop();
        return;
    }

    float spdL = fabsf(L) * MAX_SPEED;
    float spdR = fabsf(R) * MAX_SPEED;
    motor1.setMaxSpeed(spdL > 0.0f ? spdL : 1.0f);
    motor2.setMaxSpeed(spdR > 0.0f ? spdR : 1.0f);
    motor1.moveTo(motor1.currentPosition() + (L >= 0.0f ?  RUN_TARGET_STEPS : -RUN_TARGET_STEPS));
    motor2.moveTo(motor2.currentPosition() + (R >= 0.0f ?  RUN_TARGET_STEPS : -RUN_TARGET_STEPS));
    throttleMode = true;
}

// ── Setup ─────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 Dual RMCS-2301 Motor Controller + PS5 / ROS2 Teleop");

    pinMode(M1_STEP_PIN, OUTPUT); digitalWrite(M1_STEP_PIN, LOW);
    pinMode(M1_DIR_PIN,  OUTPUT); digitalWrite(M1_DIR_PIN,  LOW);
    pinMode(M2_STEP_PIN, OUTPUT); digitalWrite(M2_STEP_PIN, LOW);
    pinMode(M2_DIR_PIN,  OUTPUT); digitalWrite(M2_DIR_PIN,  LOW);

    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    enc1_state = ((uint8_t)digitalRead(ENC1_A) << 1) | (uint8_t)digitalRead(ENC1_B);
    enc2_state = ((uint8_t)digitalRead(ENC2_A) << 1) | (uint8_t)digitalRead(ENC2_B);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), enc2_isr, CHANGE);

    Serial.println("Encoders: 4x quadrature on GPIO 18/19 (M1) and GPIO 32/33 (M2)");

    Serial2.begin(115200, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    Serial.printf("UART2 bridge: RX=GPIO%d  TX=GPIO%d  @ 115200\n",
                  UART2_RX_PIN, UART2_TX_PIN);

    motor1.setMaxSpeed(MAX_SPEED);
    motor1.setAcceleration(DEFAULT_ACCEL);
    motor1.setMinPulseWidth(MIN_PULSE_WIDTH_US);
    motor1.setCurrentPosition(0);

    motor2.setMaxSpeed(MAX_SPEED);
    motor2.setAcceleration(DEFAULT_ACCEL);
    motor2.setMinPulseWidth(MIN_PULSE_WIDTH_US);
    motor2.setCurrentPosition(0);

    printConfig();
    printHelp();

    Serial.println("Waiting for RPi bridge on UART2...");
    Serial2.println("BOOTING");

    uint32_t waitStart = millis();
    bool     rpiSeen   = false;
    while (!rpiSeen) {
        if (Serial2.available()) {
            rpiSeen = true;
            while (Serial2.available()) Serial2.read();
            Serial.println("RPi bridge detected on UART2.");
        }
        if ((millis() - waitStart) >= RPI_WAIT_TIMEOUT_MS) {
            Serial.println("RPi wait timeout — continuing without ROS2.");
            break;
        }
        if (((millis() - waitStart) % 2000) < 50) {
            Serial.printf("  ... waiting for RPi  (%lu s elapsed)\n",
                          (millis() - waitStart) / 1000UL);
            Serial2.println("BOOTING");
        }
        delay(10);
    }

    Serial.printf("M1: STEP=GPIO%d  DIR=GPIO%d\n", M1_STEP_PIN, M1_DIR_PIN);
    Serial.printf("M2: STEP=GPIO%d  DIR=GPIO%d\n", M2_STEP_PIN, M2_DIR_PIN);

    g_lastEncMs = millis();
    Serial2.println("READY");
    Serial.println("READY — waiting for J commands");
}

// ── Info ──────────────────────────────────────────────────────────────────

void printConfig() {
    Serial.println("=========================================");
    Serial.printf("Wheel radius         : %.3f m  (%.1f mm)\n",
                  WHEEL_RADIUS_M, WHEEL_DIAMETER_MM / 2.0f);
    Serial.printf("Wheel diameter       : %.1f mm\n",  WHEEL_DIAMETER_MM);
    Serial.printf("Wheel circumference  : %.2f mm\n",  WHEEL_CIRCUMFERENCE_MM);
    Serial.printf("Wheelbase            : %.0f mm\n",  WHEELBASE_MM);
    Serial.printf("Steps / wheel rev    : %ld\n",      STEPS_PER_WHEEL_REV);
    Serial.printf("Steps per mm         : %.3f\n",     STEPS_PER_MM);
    Serial.printf("Max speed            : %.0f steps/s  (%.1f mm/s)\n",
                  MAX_SPEED, MAX_SPEED / STEPS_PER_MM);
    Serial.printf("Acceleration         : %.0f steps/s^2\n", DEFAULT_ACCEL);
    Serial.printf("Watchdog timeout     : %d ms\n",    WATCHDOG_MS);
    Serial.printf("Encoder report rate  : %d Hz\n",    ENC_REPORT_HZ);
    Serial.println("Encoders             : 4x quadrature, GPIO 18/19 (M1) 32/33 (M2)");
    Serial.println("=========================================");
}

void printHelp() {
    Serial.println();
    Serial.println("=========================================");
    Serial.println("  COMMANDS  (USB serial or UART2 / ROS2)");
    Serial.println("=========================================");
    Serial.println("  STRAIGHT MOTION");
    Serial.println("    fwd <mm>           Both forward      e.g. fwd 300");
    Serial.println("    bwd <mm>           Both backward     e.g. bwd 150");
    Serial.println();
    Serial.println("  ROBOT ROTATION (pivot between wheels)");
    Serial.println("    left <deg>         Turn CCW          e.g. left 90");
    Serial.println("    right <deg>        Turn CW           e.g. right 45");
    Serial.println();
    Serial.println("  WHEEL ROTATION (by wheel angle)");
    Serial.println("    rotboth <deg>      Both wheels spin  e.g. rotboth 360");
    Serial.println("    rotleft <deg>      Left wheel only   e.g. rotleft 180");
    Serial.println("    rotright <deg>     Right wheel only  e.g. rotright -90");
    Serial.println();
    Serial.println("  CONTINUOUS (smooth accel/decel ramp)");
    Serial.println("    run <L> <R>        Throttle -1.0..+1.0 per wheel");
    Serial.println("                       e.g. run 0.5 0.5   both forward");
    Serial.println("                       e.g. run 0.4 -0.4  spin left");
    Serial.println("                       send 'stop' to ramp down");
    Serial.println();
    Serial.println("  STOP");
    Serial.println("    stop               Smooth decel stop both motors");
    Serial.println();
    Serial.println("  ROS2 / PS5 CONTROL (UART2)");
    Serial.println("    J <ly> <rx>        Joystick -100..+100 (tank-drive)");
    Serial.println("    S                  Immediate hard stop");
    Serial.println();
    Serial.println("  DIAGNOSTICS");
    Serial.println("    enc                Print current encoder counts");
    Serial.println("    encreset           Zero encoder counts");
    Serial.println("    m1test             Motor 1 — 500 steps forward");
    Serial.println("    m2test             Motor 2 — 500 steps forward");
    Serial.println("    pinstate           STEP / DIR pin states");
    Serial.println("    pos                Step positions");
    Serial.println("    info               Drive-train config");
    Serial.println("    help               This list");
    Serial.println();
    Serial.println("  SETTINGS");
    Serial.println("    speed <mm/s>       e.g. speed 30");
    Serial.println("    accel <steps/s2>   e.g. accel 500");
    Serial.println("    driverreset        Zero step positions");
    Serial.println("=========================================");
    Serial.println();
}

// ── Serial Command Handler ────────────────────────────────────────────────

void handleCommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    while (line.length() > 0 && !isalpha((unsigned char)line[0])) {
        line.remove(0, 1);
    }
    if (line.length() == 0) return;

    char fc = line[0];
    if (isdigit((unsigned char)fc) || fc == '-' || fc == '.') return;

    int    sp   = line.indexOf(' ');
    String cmd  = (sp == -1) ? line : line.substring(0, sp);
    String args = (sp == -1) ? ""   : line.substring(sp + 1);
    cmd.toLowerCase();

    if (cmd == "j") {
        if (args.length() == 0) return;
        float ly = 0.0f, rx = 0.0f;
        if (sscanf(args.c_str(), "%f %f", &ly, &rx) != 2) return;
        ly = constrain(ly, -100.0f, 100.0f);
        rx = constrain(rx, -100.0f, 100.0f);
        g_lastCmdMs     = millis();
        g_watchdogArmed = true;
        applyJoystick(ly, rx);
        return;
    }

    if (cmd == "s") {
        fullStop();
        Serial2.println("OK STOPPED");
        Serial.println(">> Hard stop (S command)");
        return;
    }

    if (cmd == "fwd") {
        float mm = args.toFloat();
        if (mm <= 0) { Serial.println("ERR: fwd needs positive mm"); return; }
        Serial.printf(">> Forward %.1f mm\n", mm);
        moveDistance(mm);
        Serial.println("   Done.");

    } else if (cmd == "bwd") {
        float mm = args.toFloat();
        if (mm <= 0) { Serial.println("ERR: bwd needs positive mm"); return; }
        Serial.printf(">> Backward %.1f mm\n", mm);
        moveDistance(-mm);
        Serial.println("   Done.");

    } else if (cmd == "left") {
        float deg = args.toFloat();
        Serial.printf(">> Rotate left (CCW) %.1f deg\n", deg);
        rotateRobot(deg);
        Serial.println("   Done.");

    } else if (cmd == "right") {
        float deg = args.toFloat();
        Serial.printf(">> Rotate right (CW) %.1f deg\n", deg);
        rotateRobot(-deg);
        Serial.println("   Done.");

    } else if (cmd == "rotboth") {
        float deg = args.toFloat();
        if (deg == 0) { Serial.println("ERR: rotboth needs non-zero degrees"); return; }
        long steps = (long)((deg / 360.0f) * (float)STEPS_PER_WHEEL_REV);
        Serial.printf(">> Both wheels %.1f deg  (%ld steps)\n", deg, steps);
        rotWheelBoth(deg);
        Serial.println("   Done.");

    } else if (cmd == "rotleft") {
        float deg = args.toFloat();
        if (deg == 0) { Serial.println("ERR: rotleft needs non-zero degrees"); return; }
        Serial.printf(">> Left wheel %.1f deg  (right stationary)\n", deg);
        rotWheelLeft(deg);
        Serial.println("   Done.");

    } else if (cmd == "rotright") {
        float deg = args.toFloat();
        if (deg == 0) { Serial.println("ERR: rotright needs non-zero degrees"); return; }
        Serial.printf(">> Right wheel %.1f deg  (left stationary)\n", deg);
        rotWheelRight(deg);
        Serial.println("   Done.");

    } else if (cmd == "stop") {
        fullStop();
        Serial.println(">> Both motors stopped.");

    } else if (cmd == "run") {
        float L = 0.0f, R = 0.0f;
        sscanf(args.c_str(), "%f %f", &L, &R);
        L = constrain(L, -1.0f, 1.0f);
        R = constrain(R, -1.0f, 1.0f);
        if (L == 0.0f && R == 0.0f) {
            fullStop();
            Serial.println(">> Throttle 0 — motors stopping.");
        } else {
            float spdL = fabsf(L) * MAX_SPEED;
            float spdR = fabsf(R) * MAX_SPEED;
            motor1.setMaxSpeed(spdL > 0.0f ? spdL : 1.0f);
            motor2.setMaxSpeed(spdR > 0.0f ? spdR : 1.0f);
            motor1.moveTo(motor1.currentPosition() + (L >= 0.0f ?  RUN_TARGET_STEPS : -RUN_TARGET_STEPS));
            motor2.moveTo(motor2.currentPosition() + (R >= 0.0f ?  RUN_TARGET_STEPS : -RUN_TARGET_STEPS));
            throttleMode = true;
            Serial.printf(">> Throttle  L=%.2f  R=%.2f  (send 'stop' to ramp down)\n", L, R);
        }

    } else if (cmd == "speed") {
        float mmps = args.toFloat();
        if (mmps <= 0) { Serial.println("ERR: speed must be > 0"); return; }
        setSpeedMMperS(mmps);
        Serial.printf(">> Speed set to %.1f mm/s  (%.0f steps/s)\n",
                      mmps, mmps * STEPS_PER_MM);

    } else if (cmd == "accel") {
        float a = args.toFloat();
        if (a <= 0) { Serial.println("ERR: accel must be > 0"); return; }
        setAcceleration(a);
        Serial.printf(">> Acceleration set to %.0f steps/s^2\n", a);

    } else if (cmd == "driverreset") {
        Serial.println(">> Resetting driver faults...");
        resetDriverFault();
        Serial.println(">> Done.");

    } else if (cmd == "enc") {
        noInterrupts();
        long e1 = enc1_count;
        long e2 = enc2_count;
        interrupts();
        Serial.printf(">> Enc1 (LEFT)  = %ld\n", e1);
        Serial.printf(">> Enc2 (RIGHT) = %ld\n", e2);

    } else if (cmd == "encreset") {
        noInterrupts();
        enc1_count = 0;
        enc2_count = 0;
        interrupts();
        Serial.println(">> Encoder counts zeroed.");

    } else if (cmd == "m1test") {
        Serial.println(">> Motor 1 test — 500 steps forward...");
        motor1.setSpeed(0);
        motor1.move(500);
        while (motor1.isRunning()) motor1.run();
        motor1.setSpeed(0);
        delay(500);
        Serial.println("   Motor 1 done.");

    } else if (cmd == "m2test") {
        Serial.println(">> Motor 2 test — 500 steps forward...");
        motor2.setSpeed(0);
        motor2.move(500);
        while (motor2.isRunning()) motor2.run();
        motor2.setSpeed(0);
        delay(500);
        Serial.println("   Motor 2 done.");

    } else if (cmd == "pinstate") {
        Serial.println("-- Pin States ----------------------------");
        Serial.printf("   M1 STEP (GPIO %d): %s\n", M1_STEP_PIN,
                      digitalRead(M1_STEP_PIN) ? "HIGH" : "LOW");
        Serial.printf("   M1 DIR  (GPIO %d): %s\n", M1_DIR_PIN,
                      digitalRead(M1_DIR_PIN)  ? "HIGH" : "LOW");
        Serial.printf("   M2 STEP (GPIO %d): %s\n", M2_STEP_PIN,
                      digitalRead(M2_STEP_PIN) ? "HIGH" : "LOW");
        Serial.printf("   M2 DIR  (GPIO %d): %s\n", M2_DIR_PIN,
                      digitalRead(M2_DIR_PIN)  ? "HIGH" : "LOW");
        Serial.println("-----------------------------------------");

    } else if (cmd == "info") {
        printConfig();

    } else if (cmd == "pos") {
        Serial.printf(">> M1 (LEFT)  step pos: %ld\n", motor1.currentPosition());
        Serial.printf(">> M2 (RIGHT) step pos: %ld\n", motor2.currentPosition());

    } else if (cmd == "help") {
        printHelp();

    } else {
        Serial.printf("ERR: unknown command '%s'  (type help)\n", cmd.c_str());
    }
}

// ── Main Loop ─────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();

    if (throttleMode) {
        motor1.run();
        motor2.run();
    }

    if (g_watchdogArmed && (now - g_lastCmdMs) > WATCHDOG_MS) {
        g_watchdogArmed = false;
        fullStop();
        Serial2.println("OK STOPPED");
        Serial.println("Watchdog: no J command — motors stopped");
    }

    if ((now - g_lastEncMs) >= ENC_REPORT_MS) {
        g_lastEncMs = now;
        noInterrupts();
        long e1 = enc1_count;
        long e2 = enc2_count;
        interrupts();
        Serial2.printf("E %ld %ld\r\n", e1, e2);
    }

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBuf.length() > 0) {
                Serial.print("> ");
                Serial.println(serialBuf);
                handleCommand(serialBuf);
                serialBuf = "";
            }
        } else {
            serialBuf += c;
        }
    }

    while (Serial2.available()) {
        char c = (char)Serial2.read();
        if (c == '\n' || c == '\r') {
            if (serialBuf2.length() > 0) {
                handleCommand(serialBuf2);
                serialBuf2 = "";
            }
        } else {
            serialBuf2 += c;
        }
    }
}
