#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "KalmanFilter.h"

MPU6050 accelgyro;

struct RAW_type{
    uint8_t x;
    uint8_t y;
    uint8_t z;
};

int16_t ax, ay, az;
int16_t gx, gy, gz;
struct RAW_type accel_self_enable ,accel_self_diable;
struct RAW_type accel_zero_offsent ,gyro_zero_offsent;

#define LED_PIN 13   //NANO LED PIN
bool blinkState = false;

const float pi = 3.1415926;
const float Rad = 57.3 ;  //180.0/pi;
int w = 0;
float tmpf = 0.0;
int currentTime, signRzGyro;
int lastTime = 0;
int interval = 0;
float wGyro = 10.0;

float RwAcc[3];         // ACC raw data
float Gyro[3];          // 
float RwGyro[3];        // Gyro raw dat
float Awz[2];           // 
float RwEst[3];

float AcceRatio = 16384.0;
float GyroRatio = 131.0*57.3;

float accx, accy, accz;
float gyrox, gyroy, gyroz;

float roll = 0, pitch = 0, yaw = 0;
KalmanFilter mKalFilter;

void setup()
{
    int i ;
    int32_t ax_zero = 0,ay_zero = 0,az_zero = 0,gx_zero =0 ,gy_zero = 0,gz_zero = 0 ;
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Serial.begin(9600);
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    delay(500) ;
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    // enable Accel Self Test
    accelgyro.setAccelXSelfTest(1); // enable accel self test
    accelgyro.setAccelYSelfTest(1);
    accelgyro.setAccelZSelfTest(1);
    delay(500);
    accel_self_enable.x = accelgyro.getAccelXSelfTestDate();
    accel_self_enable.y = accelgyro.getAccelYSelfTestDate();
    accel_self_enable.z = accelgyro.getAccelZSelfTestDate();

    accelgyro.setAccelXSelfTest(0); // disable accel self test
    accelgyro.setAccelYSelfTest(0);
    accelgyro.setAccelZSelfTest(0);
    accel_self_diable.x = accelgyro.getAccelXSelfTestDate();
    accel_self_diable.y = accelgyro.getAccelYSelfTestDate();
    accel_self_diable.z = accelgyro.getAccelZSelfTestDate();

    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    /* get zero accel/gyro value */
    for( i = 0 ; i < 200 ; i++){
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //get raw accel/gyro measurements 
        ax_zero += az;
        ay_zero += ay;
        az_zero +=(az - 16384);
        gx_zero += gx;
        gy_zero += gy;
        gz_zero += gz;
    }
    accel_zero_offsent.x = ax_zero/200;
    accel_zero_offsent.y = ay_zero/200;
    accel_zero_offsent.z = az_zero/200;
    gyro_zero_offsent.x = gx_zero/200;
    gyro_zero_offsent.y = gy_zero/200;
    gyro_zero_offsent.z = gz_zero/200;
    pinMode(LED_PIN, OUTPUT);
}

void getInclination()
{
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    mKalFilter.Angle_Y(ax - accel_zero_offsent.x, ay - accel_zero_offsent.y, az-accel_zero_offsent.z, gx - gyro_zero_offsent.x, gy - gyro_zero_offsent.y, gz - gyro_zero_offsent.z);
    pitch = mKalFilter.angle /57.3 ;
}

void loop() {
    // get inclination measurements from device
    getInclination();
    Serial.print(pitch);Serial.print(",");
    Serial.print(roll);Serial.print(",");
    Serial.print(yaw); Serial.print("\n");
    //delay(5);
}
