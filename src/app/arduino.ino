//Dont think you need this first one really
//I was having problems with my arduino crapping out
#include "printf.h"
//library for gyro/acc/and magnetometer

//Dont think you will be needing this
//#include "Kalman.h" // Source: https://github.c
#include "Wire.h"

/*
 * Spencer, you may be forced to use the library for your chip
 * (9025) but i think it will let you use this one, and i would
 * recomend it for now.It will save you the hassel of changing
 * a bunch of shit at the outset
 */
#include "MPU9250.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#endif

MPU9250 mpu;

//this chooses your chip function (See line 156ish)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

volatile bool mpuInterrupt = true; // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}

int i = 0; // counter for initializing angles
int ii = 0;
int runs = 1;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//#define trigPin 7   //triggr pin for top sonar
//#define echoPin 8   //echo pin for top sonar
#define RX_PIN A0 //recieves motor input data

void setup()
{
    delay(100);
    TWBR = 24; // Set I2C frequency to 400kHz

    i2cData[0] = 0;    //7 Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x06; //0x00 Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x01; //0x00 Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x01; //0x00 Set Accelerometer Full Scale Range to ±2g
    delay(100);
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    Serial.begin(115200);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    wdt_enable(WDTO_250MS);
    Serial.flush();
}

void loop()
{

    //  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    //        // reset so we can continue cleanly
    //        mpu.resetFIFO();
    //        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    //}
    if (runs == 1)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //        #ifdef OUTPUT_READABLE_QUATERNION
        //            // display quaternion values in easy matrix form: w x y z
        //            mpu.dmpGetQuaternion(&q, fifoBuffer);
        //        #endif
        //
        //        #ifdef OUTPUT_READABLE_EULER
        //            // display Euler angles in degrees
        //            mpu.dmpGetQuaternion(&q, fifoBuffer);
        //            mpu.dmpGetEuler(euler, &q);
        //        #endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif

        //        #ifdef OUTPUT_READABLE_REALACCEL
        //            // display real acceleration, adjusted to remove gravity
        //            mpu.dmpGetQuaternion(&q, fifoBuffer);
        //            mpu.dmpGetAccel(&aa, fifoBuffer);
        //            mpu.dmpGetGravity(&gravity, &q);
        //            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //        #endif

        //        #ifdef OUTPUT_READABLE_WORLDACCEL
        //            // display initial world-frame acceleration, adjusted to remove gravity
        //            // and rotated based on known orientation from quaternion
        //            mpu.dmpGetQuaternion(&q, fifoBuffer);
        //            mpu.dmpGetAccel(&aa, fifoBuffer);
        //            mpu.dmpGetGravity(&gravity, &q);
        //            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //        #endif

        //        #ifdef OUTPUT_TEAPOT
        //            // display quaternion values in InvenSense Teapot demo format:
        //            teapotPacket[2] = fifoBuffer[0];
        //            teapotPacket[3] = fifoBuffer[1];
        //            teapotPacket[4] = fifoBuffer[4];
        //            teapotPacket[5] = fifoBuffer[5];
        //            teapotPacket[6] = fifoBuffer[8];
        //            teapotPacket[7] = fifoBuffer[9];
        //            teapotPacket[8] = fifoBuffer[12];
        //            teapotPacket[9] = fifoBuffer[13];
        //
        //            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        //        #endif
        //    }

        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = ypr[2] * 180 / M_PI;

        Serial.print(ypr[0]);
        Serial.print(ypr[1]);
        Serial.print(ypr[2]);
        Serial.println();

        wdt_reset();
    }
}
