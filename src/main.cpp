#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <MS5837.h>

// ================= SENSOR =================
Adafruit_MPU6050 imu;
Adafruit_HMC5883_Unified hmc5883(12345);
MS5837 depthSensor;

// ================= MAHONY FILTER =================
float twoKp = 2.0f * 0.5f;
float twoKi = 2.0f * 0.0f;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float integralFBx = 0;
float integralFBy = 0;
float integralFBz = 0;

unsigned long lastTime;

// =================================================
void MahonyUpdate(
  float gx, float gy, float gz,
  float ax, float ay, float az,
  float mx, float my, float mz,
  float dt)
{
  float recipNorm;
  float hx, hy, bx, bz;
  float vx, vy, vz;
  float wx, wy, wz;
  float ex, ey, ez;

  if((ax!=0)||(ay!=0)||(az!=0))
  {
    recipNorm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
    ax*=recipNorm; ay*=recipNorm; az*=recipNorm;

    recipNorm = 1.0f / sqrt(mx*mx + my*my + mz*mz);
    mx*=recipNorm; my*=recipNorm; mz*=recipNorm;

    hx = 2*mx*(0.5f - q2*q2 - q3*q3)
       + 2*my*(q1*q2 - q0*q3)
       + 2*mz*(q1*q3 + q0*q2);

    hy = 2*mx*(q1*q2 + q0*q3)
       + 2*my*(0.5f - q1*q1 - q3*q3)
       + 2*mz*(q2*q3 - q0*q1);

    bx = sqrt((hx*hx)+(hy*hy));
    bz = 2*mx*(q1*q3 - q0*q2)
       + 2*my*(q2*q3 + q0*q1)
       + 2*mz*(0.5f - q1*q1 - q2*q2);

    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5f - q1*q1 - q2*q2);

    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    integralFBx += twoKi * ex * dt;
    integralFBy += twoKi * ey * dt;
    integralFBz += twoKi * ez * dt;

    gx += integralFBx + twoKp*ex;
    gy += integralFBy + twoKp*ey;
    gz += integralFBz + twoKp*ez;
  }

  gx *= (0.5f*dt);
  gy *= (0.5f*dt);
  gz *= (0.5f*dt);

  float qa=q0, qb=q1, qc=q2;

  q0 += (-qb*gx - qc*gy - q3*gz);
  q1 += (qa*gx + qc*gz - q3*gy);
  q2 += (qa*gy - qb*gz + q3*gx);
  q3 += (qa*gz + qb*gy - qc*gx);

  recipNorm = 1.0f / sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

  q0*=recipNorm;
  q1*=recipNorm;
  q2*=recipNorm;
  q3*=recipNorm;
}

// =================================================
void setup() {

  Serial.begin(115200);
  Wire.begin();

  imu.begin();

  Wire.beginTransmission(0x68);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(100);

  hmc5883.begin();

  depthSensor.init();
  depthSensor.setFluidDensity(997);

  lastTime = millis();

  Serial.println("Mahony AHRS Ready");
}

// =================================================
void loop() {

  float dt = (millis()-lastTime)/1000.0f;
  lastTime = millis();

  sensors_event_t a,g,temp;
  imu.getEvent(&a,&g,&temp);

  sensors_event_t mag;
  hmc5883.getEvent(&mag);

  MahonyUpdate(
    g.gyro.x, g.gyro.y, g.gyro.z,
    a.acceleration.x, a.acceleration.y, a.acceleration.z,
    mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
    dt
  );

  // Euler conversion
  float roll  = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))*RAD_TO_DEG;
  float pitch = asin(2*(q0*q2-q3*q1))*RAD_TO_DEG;
  float yaw   = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))*RAD_TO_DEG;

  if(yaw<0) yaw+=360;

  depthSensor.read();
  float depth = depthSensor.depth();

  Serial.print("P:");
  Serial.print(pitch,1);
  Serial.print(" R:");
  Serial.print(roll,1);
  Serial.print(" Y:");
  Serial.print(yaw,1);
  Serial.print(" D:");
  Serial.println(depth,3);

  
  delay(10);
}