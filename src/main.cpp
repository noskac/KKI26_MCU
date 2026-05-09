// ============================================================
//  ROV Main Controller
//  Board  : Teensy 4.1
//  IDE    : PlatformIO
//  Sensor : MPU6050 (IMU) + HMC5883 (Compass) + MS5837 (Depth)
//  Serial : Format dari Python → S:val,Y:val,H:val,R:val,T:val
//
//  Key Command:
//    S = Surge  (maju/mundur)        → TBKIRI, TBKANAN
//    Y = Yaw    (putar kanan/kiri)   → TBKIRI, TBKANAN
//    H = Heave  (naik/turun)         → semua vertikal
//    R = Roll   (guling kanan/kiri)  → vertikal kiri vs kanan
//    T = Tilt   (pitch depan/bawah)  → vertikal depan vs belakang
//
//  Format Serial Output:
//  Baris 1 → "P:xx.x R:xx.x Y:xx.x D:x.xxx"        → sensor (ke GUI Python)
//  Baris 2 → "CMD S:xxxx Y:xxxx H:xxxx R:xxxx T:xxxx" → echo command masuk
//  Baris 3 → "PWM DKIRI:xxxx DKANAN:xxxx ..."        → output PWM per channel
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MS5837.h>
#include <Servo.h>

// ── PWM ──────────────────────────────────────────────────────
#define PWM_MIN           1100
#define PWM_MAX           1900
#define PWM_NEUTRAL       1500
#define SERIAL_TIMEOUT_MS  500   // reset ke netral jika tidak ada data dari Python

// ── Pin Thruster ──────────────────────────────────────────────
const uint8_t PIN_DKIRI   = 8;  //A1
const uint8_t PIN_DKANAN  = 9;  //B1
const uint8_t PIN_BKIRI   = 10; //A2 
const uint8_t PIN_BKANAN  = 11; //B2
const uint8_t PIN_TKIRI  = 12;  //A3
const uint8_t PIN_TKANAN = 24;  //B3

// ── Struct ────────────────────────────────────────────────────
struct AutoCommand {
    int surge = PWM_NEUTRAL;   // S — maju/mundur        (TBKIRI, TBKANAN)
    int yaw   = PWM_NEUTRAL;   // Y — putar kanan/kiri   (TBKIRI, TBKANAN)
    int heave = PWM_NEUTRAL;   // H — naik/turun         (semua vertikal)
    int roll  = PWM_NEUTRAL;   // R — guling kanan/kiri  (vertikal kiri vs kanan)
    int tilt  = PWM_NEUTRAL;   // T — pitch depan/bawah  (vertikal depan vs belakang)
};

struct ThrusterOutput {
    int DKIRI, DKANAN, BKIRI, BKANAN, TKIRI, TKANAN;
};

// ── Thruster Class ────────────────────────────────────────────
class Thruster {
public:
    void attach(uint8_t pin, bool reversed = false) {
        this->reversed = reversed;
        esc.attach(pin, PWM_MIN, PWM_MAX);
        esc.writeMicroseconds(PWM_NEUTRAL);
    }

    void write(int value) {
        value = constrain(value, PWM_MIN, PWM_MAX);
        if (reversed) value = PWM_NEUTRAL * 2 - value;
        esc.writeMicroseconds(constrain(value, PWM_MIN, PWM_MAX));
    }

private:
    Servo esc;
    bool  reversed = false;
};

// ── Global Objects ────────────────────────────────────────────
Adafruit_MPU6050         imu;
Adafruit_HMC5883_Unified hmc5883(12345);
MS5837                   depthSensor;

Thruster tDKIRI, tDKANAN, tBKIRI, tBKANAN, tTKIRI, tTKANAN;

AutoCommand   autoCmd;
unsigned long lastAutoSerial = 0;

// ── Mahony Filter State ───────────────────────────────────────
float twoKp = 2.0f * 0.5f;
float twoKi = 2.0f * 0.0f;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0, integralFBy = 0, integralFBz = 0;

unsigned long lastTime;

// ── Forward Declarations ──────────────────────────────────────
void           MahonyUpdate(float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz, float dt);
ThrusterOutput mixing(const AutoCommand& cmd);
void           applyOutput(const ThrusterOutput& out);
bool           readCommand();

// ════════════════════════════════════════════════════════════
//  MAHONY AHRS
// ════════════════════════════════════════════════════════════
void MahonyUpdate(float gx, float gy, float gz,
                  float ax, float ay, float az,
                  float mx, float my, float mz, float dt)
{
    float recipNorm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;

    if ((ax != 0) || (ay != 0) || (az != 0)) {
        recipNorm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        recipNorm = 1.0f / sqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

        hx = 2*mx*(0.5f - q2*q2 - q3*q3)
           + 2*my*(q1*q2 - q0*q3)
           + 2*mz*(q1*q3 + q0*q2);

        hy = 2*mx*(q1*q2 + q0*q3)
           + 2*my*(0.5f - q1*q1 - q3*q3)
           + 2*mz*(q2*q3 - q0*q1);

        bx = sqrt(hx*hx + hy*hy);
        bz = 2*mx*(q1*q3 - q0*q2)
           + 2*my*(q2*q3 + q0*q1)
           + 2*mz*(0.5f - q1*q1 - q2*q2);

        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
        wy = 2*bx*(q1*q2 - q0*q3)         + 2*bz*(q0*q1 + q2*q3);
        wz = 2*bx*(q0*q2 + q1*q3)         + 2*bz*(0.5f - q1*q1 - q2*q2);

        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        integralFBx += twoKi * ex * dt;
        integralFBy += twoKi * ey * dt;
        integralFBz += twoKi * ez * dt;

        gx += integralFBx + twoKp * ex;
        gy += integralFBy + twoKp * ey;
        gz += integralFBz + twoKp * ez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

// ════════════════════════════════════════════════════════════
//  MIXING
//  Input  : PWM mentah 1100-1900, neutral = 1500
//  Output : PWM per channel setelah mixing
//
//  Horizontal:
//    TBKIRI  = surge + yaw
//    TBKANAN = surge - yaw
//
//  Vertikal (kombinasi heave + roll + tilt):
//    DKIRI  = heave - roll + tilt   (depan kiri)
//    DKANAN = heave + roll + tilt   (depan kanan)
//    BKIRI  = heave - roll - tilt   (belakang kiri)
//    BKANAN = heave + roll - tilt   (belakang kanan)
//
//  Roll (+)  → kanan naik, kiri turun
//  Roll (-)  → sebaliknya
//  Tilt (+)  → depan naik, belakang turun (nose up)
//  Tilt (-)  → depan turun, belakang naik (nose down)
// ════════════════════════════════════════════════════════════
ThrusterOutput mixing(const AutoCommand& cmd)
{
    ThrusterOutput out;

    int surge = constrain(cmd.surge - PWM_NEUTRAL, -400, 400);
    int yaw   = constrain(cmd.yaw   - PWM_NEUTRAL, -400, 400);
    int heave = constrain(cmd.heave - PWM_NEUTRAL, -400, 400);
    int roll  = constrain(cmd.roll  - PWM_NEUTRAL, -400, 400);
    int tilt  = constrain(cmd.tilt  - PWM_NEUTRAL, -400, 400);

    // Horizontal
    out.TKIRI  = constrain(PWM_NEUTRAL + surge + yaw, PWM_MIN, PWM_MAX);
    out.TKANAN = constrain(PWM_NEUTRAL + surge - yaw, PWM_MIN, PWM_MAX);

    // Vertikal
    out.DKIRI   = constrain(PWM_NEUTRAL + heave - roll + tilt, PWM_MIN, PWM_MAX);
    out.DKANAN  = constrain(PWM_NEUTRAL + heave + roll + tilt, PWM_MIN, PWM_MAX);
    out.BKIRI   = constrain(PWM_NEUTRAL + heave - roll - tilt, PWM_MIN, PWM_MAX);
    out.BKANAN  = constrain(PWM_NEUTRAL + heave + roll - tilt, PWM_MIN, PWM_MAX);

    return out;
}

// ════════════════════════════════════════════════════════════
//  APPLY OUTPUT KE THRUSTER
// ════════════════════════════════════════════════════════════
void applyOutput(const ThrusterOutput& out)
{
    tDKIRI.write(out.DKIRI);
    tDKANAN.write(out.DKANAN);
    tBKIRI.write(out.BKIRI);
    tBKANAN.write(out.BKANAN);
    tTKIRI.write(out.TKIRI);
    tTKANAN.write(out.TKANAN);
}

// ════════════════════════════════════════════════════════════
//  BACA SERIAL
//  Format dari Python : S:1500,Y:1500,H:1500,R:1500,T:1500
//  Rentang nilai      : 1100 (min) - 1500 (netral) - 1900 (max)
// ════════════════════════════════════════════════════════════
bool readCommand()
{
    if (!Serial.available()) return false;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return false;

    AutoCommand tmp       = autoCmd;
    bool        anyParsed = false;

    int start = 0;
    while (start < (int)line.length()) {
        int    comma = line.indexOf(',', start);
        String token = (comma >= 0)
                       ? line.substring(start, comma)
                       : line.substring(start);
        token.trim();

        int colon = token.indexOf(':');
        if (colon > 0) {
            char key = token.charAt(0);
            int  val = constrain(token.substring(colon + 1).toInt(),
                                 PWM_MIN, PWM_MAX);
            switch (key) {
                case 'S': tmp.surge = val; anyParsed = true; break;
                case 'Y': tmp.yaw   = val; anyParsed = true; break;
                case 'H': tmp.heave = val; anyParsed = true; break;
                case 'R': tmp.roll  = val; anyParsed = true; break;
                case 'T': tmp.tilt  = val; anyParsed = true; break;
            }
        }

        if (comma < 0) break;
        start = comma + 1;
    }

    if (anyParsed) {
        autoCmd        = tmp;
        lastAutoSerial = millis();
        return true;
    }
    return false;
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);

    tDKIRI.attach(PIN_DKIRI, false);
    tDKANAN.attach(PIN_DKANAN, false);
    tBKIRI.attach(PIN_BKIRI, true);
    tBKANAN.attach(PIN_BKANAN, false);
    tTKIRI.attach(PIN_TKIRI, false);
    tTKANAN.attach(PIN_TKANAN, false);

    Wire.begin();
    imu.begin();

    // Bypass I2C MPU6050 agar HMC5883 bisa diakses langsung
    Wire.beginTransmission(0x68);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();
    delay(100);

    hmc5883.begin();

    depthSensor.init();
    depthSensor.setFluidDensity(997);   // air tawar; pakai 1025 untuk air laut

    lastTime = millis();

    Serial.println("ROV Ready");
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop()
{
    // ── Delta time ────────────────────────────────────────────
    float dt = (millis() - lastTime) / 1000.0f;
    lastTime = millis();

    // ── Baca IMU ──────────────────────────────────────────────
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);

    sensors_event_t mag;
    hmc5883.getEvent(&mag);

    MahonyUpdate(
        g.gyro.x,         g.gyro.y,         g.gyro.z,
        a.acceleration.x, a.acceleration.y, a.acceleration.z,
        mag.magnetic.x,   mag.magnetic.y,   mag.magnetic.z,
        dt
    );

    // ── Euler Angles ──────────────────────────────────────────
    float roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    float pitch = asin (2*(q0*q2 - q3*q1))                          * RAD_TO_DEG;
    float yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;
    if (yaw < 0) yaw += 360;

    // ── Baca Depth ────────────────────────────────────────────
    depthSensor.read();
    float depth = depthSensor.depth();

    // ── Baca Perintah Serial dari Python ──────────────────────
    readCommand();

    // ── Safety Timeout: tidak ada data → semua netral ─────────
    if (millis() - lastAutoSerial > SERIAL_TIMEOUT_MS) {
        autoCmd = AutoCommand();
    }

    // ── Mixing → Apply ke Thruster ────────────────────────────
    ThrusterOutput out = mixing(autoCmd);
    applyOutput(out);

    // ── Serial Output ─────────────────────────────────────────

    // Baris 1: data sensor → dibaca Python untuk GUI
    // Format: "P:xx.x R:xx.x Y:xx.x D:x.xxx"
    Serial.print("P:");  Serial.print(pitch, 1);
    Serial.print(" R:"); Serial.print(roll,  1);
    Serial.print(" Y:"); Serial.print(yaw,   1);
    Serial.print(" D:"); Serial.println(depth, 3);

    // Baris 2: echo command yang diterima dari Python
    // Format: "CMD S:xxxx Y:xxxx H:xxxx R:xxxx T:xxxx"
    Serial.print("CMD S:"); Serial.print(autoCmd.surge);
    Serial.print(" Y:");    Serial.print(autoCmd.yaw);
    Serial.print(" H:");    Serial.print(autoCmd.heave);
    Serial.print(" R:");    Serial.print(autoCmd.roll);
    Serial.print(" T:");    Serial.println(autoCmd.tilt);

    // Baris 3: nilai PWM output per channel
    // Format: "PWM DKIRI:xxxx DKANAN:xxxx BKIRI:xxxx BKANAN:xxxx TBKIRI:xxxx TBKANAN:xxxx"
    Serial.print("PWM DKIRI:");  Serial.print(out.DKIRI);
    Serial.print(" DKANAN:");    Serial.print(out.DKANAN);
    Serial.print(" BKIRI:");     Serial.print(out.BKIRI);
    Serial.print(" BKANAN:");    Serial.print(out.BKANAN);
    Serial.print(" TBKIRI:");    Serial.print(out.TKIRI);
    Serial.print(" TBKANAN:");   Serial.println(out.TKANAN);

    delay(10);
}