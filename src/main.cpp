#define M5STACK_MPU6886 

#include <Arduino.h>
#include <M5Stack.h>
#include "BMM150class.h"
#include <utility/quaternionFilters.h>
#ifndef M5STACK_MPU6886
    #include "imu/IMU.h"
#endif
#include "ble/BLE.h"
#include "ble/BLEData.h"
#include "input/ButtonCheck.h"

#define SERIAL_PRINT 1

void printSerial(const float a[], const float g[], const float m[], const float q[]);
void printLcd(const float a[], const float g[], const float m[], const float q[]);

#ifndef M5STACK_MPU6886
imu::IMU _imu;
#endif
ble::BLE _ble;
input::ButtonCheck _button;

float acc[3];
float gyro[3];
float mag[3];
float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;

float quat[4];

float temp = 0.0f;

BMM150class bmm150;
uint32_t Now = 0;
uint32_t lastUpdate =0, firstUpdate = 0;
float deltat = 0.0f , sum=0.0f;

void setup() {
    M5.begin();
    M5.Power.begin();
#ifdef M5STACK_MPU6886
    M5.IMU.Init();
    bmm150.Init();

    for (int i=0; i<3; i++) { acc[i] = 0.0F; }
    for (int i=0; i<3; i++) { gyro[i] = 0.0F; }
    for (int i=0; i<3; i++) { mag[i] = 0.0F; }
    for (int i=0; i<4; i++) { quat[i] = (i>0) ? 0.0F : 1.0F; }
#endif
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN ,BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);

    M5.Lcd.println("awaken...");
#if SERIAL_PRINT
    Serial.println("awaken...");
#endif
    if (Wire.begin()) {
        Wire.setClock(400000UL); // i2c 400kHz
        M5.Lcd.println("i2c OK!");
#if SERIAL_PRINT
        Serial.println("i2c OK!");
#endif
    } else {
        M5.Lcd.println("i2c NG!!!");
#if SERIAL_PRINT
        Serial.println("i2c NG!");
#endif
    }
#ifndef M5STACK_MPU6886
    if (_imu.Setup(50)) { // 50Hz
        M5.Lcd.println("imu OK!");
    } else {
        M5.Lcd.println("imu NG!!!");
    }
#endif
    if (_ble.Initialize() && _ble.Start()) { // 50Hz
        M5.Lcd.println("ble OK!");
#if SERIAL_PRINT
        Serial.println("ble OK!");
#endif
    } else {
        M5.Lcd.println("ble NG!");
#if SERIAL_PRINT
        Serial.println("ble NG!");
#endif
    }
#if SERIAL_PRINT
    //Serial.begin(115200);
    Serial.println("serial OK!");
    M5.Lcd.println("serial OK!");
#endif
    M5.Lcd.fillScreen(BLACK);
}

void loop() {
    // Button condition
    if (_button.containsUpdate(M5)) {
        for (int i = 0; i < INPUT_BTN_NUM; i++) {
            input::Btn btn = input::AllBtns[i];
            if (_button.isBtnUpdate(btn)) {
                input::BtnState btnState = _button.getBtnState(btn);
                uint8_t btnCode = input::toBtnCode(btn);
                uint8_t btnPress = input::toBtnPress(btnState);
                uint32_t btnPressTime = (btnPress == 0) ? _button.getBtnPressTime(btn) : 0;
#if SERIAL_PRINT
                Serial.print(btnCode); Serial.print(" "); 
                Serial.print(btnPress); Serial.print(" "); 
                Serial.println(btnPressTime);
#endif
                ble::ButtonData data;
                data.btnCode = btnCode;
                data.btnPress = btnPress;
                data.pressTime = (uint16_t)btnPressTime;
                _ble.GetButtonCharacteristic().setValue((uint8_t*)&data, BLE_BUTTON_DATA_LEN);
                _ble.GetButtonCharacteristic().notify();
            }
        }
    }
    // IMU condition

#ifndef M5STACK_MPU6886
    if (_imu.Update()) {
        ble::IMUData data;
        memcpy(&data.accX, _imu.getAcc(), sizeof(float)*3);
        memcpy(&data.gyroX, _imu.getGyro(), sizeof(float)*3);
        memcpy(&data.magX, _imu.getMag(), sizeof(float)*3);
        memcpy(&data.quatW, _imu.getQuat(), sizeof(float)*4);
        _ble.GetNineAxisCharacteristic().setValue((uint8_t*)&data, BLE_IMU_DATA_LEN);
        _ble.GetNineAxisCharacteristic().notify();
#if SERIAL_PRINT
        printSerial(_imu.getAcc(), _imu.getGyro(), _imu.getMag(), _imu.getQuat());
#endif
        printLcd(_imu.getAcc(), _imu.getGyro(), _imu.getMag(), _imu.getQuat());
    }
#else
    ble::IMUData data;
    M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
    M5.IMU.getAccelData(&acc[0],&acc[1],&acc[2]);
    bmm150.getMagnetData(&mag[0],&mag[1],&mag[2]);
    M5.IMU.getTempData(&temp);

    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;

    MadgwickQuaternionUpdate(acc[0],acc[1],acc[2],
                            gyro[0]*DEG_TO_RAD,gyro[1]*DEG_TO_RAD,gyro[2]*DEG_TO_RAD,
                            mag[1],mag[0],mag[2],deltat);

    yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                                          *(getQ() + 3)),
              *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
    pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                            *(getQ() + 2)));
    roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
                                                     *(getQ() + 3)),
               *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;
    yaw -= 8.5;
 
    memcpy(&quat,getQ(),sizeof(float)*4);

    memcpy(&data.accX, &acc, sizeof(float)*3);
    memcpy(&data.gyroX, &gyro, sizeof(float)*3);
    memcpy(&data.magX, &mag, sizeof(float)*3);
    memcpy(&data.quatW, getQ(), sizeof(float)*4);
    _ble.GetNineAxisCharacteristic().setValue((uint8_t*)&data, BLE_IMU_DATA_LEN);
    _ble.GetNineAxisCharacteristic().notify();
#if SERIAL_PRINT
    printSerial(acc, gyro, mag, quat);
#endif
    printLcd(acc, gyro, mag, quat);
#endif
     // device update
    M5.update();
}

void printLcd(const float a[], const float g[], const float m[], const float q[]) {
  M5.Lcd.setCursor(0, 20);
  M5.Lcd.printf("%6.2f  %6.2f  %6.2f      ", g[0], g[1], g[2]);
  M5.Lcd.setCursor(220, 42);
  M5.Lcd.print(" o/s");
  M5.Lcd.setCursor(0, 65);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", a[0], a[1], a[2]);
  M5.Lcd.setCursor(220, 87);
  M5.Lcd.print(" G");
  M5.Lcd.setCursor(0, 110);
  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", m[0], m[1], m[2]);
  M5.Lcd.setCursor(220, 132);
  M5.Lcd.print(" mag");
  M5.Lcd.setCursor(0, 154);
  M5.Lcd.print("  qw     qx     qy     qz");
  M5.Lcd.setCursor(0,  176);
  M5.Lcd.printf(" %2.3f % 2.3f %2.3f %2.3f \r\n", q[0], q[1], q[2], q[3]);
}

#if SERIAL_PRINT
void printSerial(const float a[], const float g[], const float m[], const float q[]) {
    return;
    Serial.print(a[0], 3); Serial.print(",");
    Serial.print(a[1], 3); Serial.print(",");
    Serial.print(a[2], 3); Serial.print(",");
    Serial.print(g[0], 3); Serial.print(",");
    Serial.print(g[1], 3); Serial.print(",");
    Serial.print(g[2], 3); Serial.print(",");
    Serial.print(m[0], 3); Serial.print(",");
    Serial.print(m[1], 3); Serial.print(",");
    Serial.print(m[2], 3); Serial.print(",");
    Serial.print(q[0], 3); Serial.print(",");
    Serial.print(q[1], 3); Serial.print(",");
    Serial.print(q[2], 3); Serial.print(",");
    Serial.print(q[3], 3); Serial.println();
}
#endif