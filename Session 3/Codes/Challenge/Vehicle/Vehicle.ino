#include <I2Cdev.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <MPU6050_6Axis_MotionApps20.h>

//#define DEBUG
//#define TEST

#define PIN_LED_SIG 3
#define PIN_RF_CE 9
#define PIN_RF_CSN 10
#define PIN_MPU_INT 2

#define MSG_R_INDEX_MODE 0
#define MSG_R_INDEX_LSPEED 1
#define MSG_R_INDEX_RSPEED 2
#define MSG_T_INDEX_ST_SLOPE 0
#define MSG_T_INDEX_DISTANCE1 1
#define MSG_T_INDEX_DISTANCE2 2

#define RF_PIPE 1
#define RF_ADDR_T 1
#define RF_ADDR_R 0
#define RF_CHANNEL 117

#define YPR_Y 0
#define YPR_P 1
#define YPR_R 2

#define SIG_LOST_TOLERANCE 11
#define SIG_RT_RATIO 5
#define SIG_RT_PERIOD 10

#define AUTO_LINE_SPEED 80
#define AUTO_CURVE_DIFF 18
#define AUTO_SPIN_MAX_SPEED 35
#define AUTO_SPIN_MIN_SPEED 20
#define AUTO_SPIN_SLOW_ANGLE 20
#define AUTO_TILE_SPEED 40
#define AUTO_MIN_ANGLE_DETECT (-20)
#define AUTO_TILT_DISTANCE 80 //cm
#define AUTO_LSPEED_OFFSET (-3)
#define AUTO_RSPEED_OFFSET (0)

#define COM_BASEBOARD_ADDRESS 42

volatile bool mode = false;                                     //false->mannual    //true->auto
volatile char messageRecieve[3] = {0x00, 0x00, 0x00};           //mode + Left speed + Right speed
volatile unsigned char messageTransmit[3] = {0x00, 0x00, 0x00}; //1bit state + 7 bit slope + 2 bytes distance

volatile bool tiltPosition = false;
volatile char leftSpeed = 0;
volatile char rightSpeed = 0;

unsigned long encoderp1 = 0;
unsigned long encoderp2 = 0;
unsigned long encoder1 = 0;
unsigned long encoder2 = 0;

volatile int yprDegree[3] = {0, 0, 0};
volatile double distance = 0;
volatile bool state = false; //true->move    //false->stop

//First address is used for message transmit
//Second address is used for message recieve
const byte address[2][5] = {"1Node", "2Node"};

volatile unsigned char lostCounter = 0;

RF24 rf24(PIN_RF_CE, PIN_RF_CSN); // CE, CSN

MPU6050 mpu;

bool dmpReady = false;        // set true if DMP init was successful
unsigned char mpuIntStatus;   // holds actual interrupt status byte from MPU
unsigned char devStatus;      // return status after each device operation (0 = success, !0 = error)
unsigned int packetSize;      // expected DMP packet size (default is 42 bytes)
unsigned int fifoCount;       // count of all bytes currently in FIFO
unsigned char fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void AutoOpreate()
{
#ifdef TEST
    Serial.println(1);
#endif
    //300cm straight line
    AutoMoveForward(290);
#ifdef TEST
    Serial.println(2);
#endif
    //90d right
    AutoTurn(true, 90);
#ifdef TEST
    Serial.println(3);
#endif
    //curve
    leftSpeed = AUTO_LINE_SPEED + AUTO_CURVE_DIFF;
    rightSpeed = AUTO_LINE_SPEED - AUTO_CURVE_DIFF;
    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    WaitDegreeTarget(170);
#ifdef TEST
    Serial.println(4);
#endif
    //300cm straight line
    if(tiltPosition)
    {
        AutoMoveForward(280);
    }
    else
    {
        AutoMoveForward(310);
    }
#ifdef TEST
    Serial.println(5);
#endif
    //90d right
    AutoTurn(true, -90);
#ifdef TEST
    Serial.println(6);
#endif
    //80cm straight line
    AutoMoveForward(60);
#ifdef TEST
    Serial.println(7);
#endif
    //135d left
    AutoTurn(true, -50);
#ifdef TEST
    Serial.println(8);
#endif
    //80√2cm straight line
    leftSpeed = -(AUTO_LINE_SPEED + AUTO_LSPEED_OFFSET);
    rightSpeed = -(AUTO_LINE_SPEED + AUTO_RSPEED_OFFSET);
    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    WaitDistanceTarget(82 * sqrt(2));
#ifdef TEST
    Serial.println(9);
#endif
    //135d left
    AutoTurn(true, 0);
#ifdef TEST
    Serial.println(10);
#endif
    //80cm straight line
    AutoMoveForward(80);
#ifdef TEST
    Serial.println(11);
#endif
    //45d left
    AutoTurn(false, -45);
#ifdef TEST
    Serial.println(12);
#endif
    //80√2cm straight line
    AutoMoveForward(74 * sqrt(2));
#ifdef TEST
    Serial.println(13);
#endif
    //45d right
    //AutoTurn(true, 0);
}

void setup()
{
#ifdef DEBUG
    Serial.begin(9600);
#endif

#ifdef TEST
    Serial.begin(9600);
#endif

    //Wire
    Wire.begin();

    //RF24
    rf24.begin();
    rf24.setChannel(RF_CHANNEL);
    rf24.setPALevel(RF24_PA_MAX);
    rf24.setDataRate(RF24_2MBPS);
    rf24.openReadingPipe(RF_PIPE, address[RF_ADDR_R]);
    rf24.openWritingPipe(address[RF_ADDR_T]);

    //Signal LED
    pinMode(PIN_LED_SIG, OUTPUT);

    //MPU6050
    mpu.initialize();
    pinMode(PIN_MPU_INT, INPUT);
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    //Initialize encoder
    ReadEncoder();
}

void loop()
{
    if (mode)
    {
        AutoOpreate();
        mode = false;
        tiltPosition = false;
        rf24.flush_rx();
    }
    else
    {
        MPUHandler();
        DistanceHandler();
        MessageGenerator();
#ifdef TEST
        Serial.print("mode:");
        Serial.println(mode);
#endif
        RFReader();
#ifdef TEST
        Serial.print("mode:");
        Serial.println(mode);
#endif
        RFWriter(true);
    }
}

void WaitDistanceTarget(double encoder) //cm
{
    double target = encoder + distance;
    while (true)
    {
        MPUHandler();
        DistanceHandler();
        MessageGenerator();
        RFWriter(false);
        //Detect tilt
#ifdef TEST
        Serial.print("Pitch degree:");
        Serial.println(yprDegree[YPR_P]);
#endif
        if (yprDegree[YPR_P] < AUTO_MIN_ANGLE_DETECT)
        {
            tiltPosition = true;
#ifdef TEST
            Serial.println("Tilt!");
#endif
            double tileTarget = AUTO_TILT_DISTANCE + distance;
            while (true)
            {
                MPUHandler();
                DistanceHandler();
                MessageGenerator();
                RFWriter(false);

                if (distance > tileTarget)
                {
                    leftSpeed = 0;
                    rightSpeed = 0;
                    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

                    delay(2000);
#ifdef TEST
                    Serial.println("Spining");
#endif
                    leftSpeed = AUTO_TILE_SPEED;
                    rightSpeed = -AUTO_TILE_SPEED;
                    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
                    if(yprDegree[YPR_Y] > -90 && yprDegree[YPR_Y] < 90)
                    {
                        WaitDegreeTarget(180);
                        WaitDegreeTarget(-20);
                    }
                    else
                    {
                        WaitDegreeTarget(0);
                        WaitDegreeTarget(160);
                    }
                    
                    leftSpeed = AUTO_LINE_SPEED + AUTO_LSPEED_OFFSET;
                    rightSpeed = AUTO_LINE_SPEED + AUTO_RSPEED_OFFSET;
                    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
                    break;
                }
            }
        }

        if (distance > target)
        {
            break;
        }
    }
}

void AutoMoveForward(double encoder)
{
    leftSpeed = AUTO_LINE_SPEED + AUTO_LSPEED_OFFSET;
    rightSpeed = AUTO_LINE_SPEED + AUTO_RSPEED_OFFSET;
    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    WaitDistanceTarget(encoder);
}

void AutoTurn(bool LR, int degree) //true->right //false->left
{
    if (LR)
    {
        leftSpeed = AUTO_SPIN_MAX_SPEED;
        rightSpeed = -AUTO_SPIN_MAX_SPEED;
    }
    else
    {
        leftSpeed = -AUTO_SPIN_MAX_SPEED;
        rightSpeed = AUTO_SPIN_MAX_SPEED;
    }

    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

    if (LR)
    {
        WaitDegreeTarget(degree - AUTO_SPIN_SLOW_ANGLE);
        leftSpeed = AUTO_SPIN_MIN_SPEED;
        rightSpeed = -AUTO_SPIN_MIN_SPEED;
    }
    else
    {
        WaitDegreeTarget(degree + AUTO_SPIN_SLOW_ANGLE);
        leftSpeed = -AUTO_SPIN_MIN_SPEED;
        rightSpeed = AUTO_SPIN_MIN_SPEED;
    }

    MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    WaitDegreeTarget(degree);
}

void WaitDegreeTarget(int degree) //degree
{
    int preYaw;
    int nowYaw;
    while (true)
    {
        preYaw = yprDegree[YPR_Y];
        MPUHandler();
        nowYaw = yprDegree[YPR_Y];
#ifdef TEST
        Serial.print("Pre");
        Serial.print(preYaw);
        Serial.print("\t");
        Serial.print("Now");
        Serial.print(nowYaw);
        Serial.print("\t");
        Serial.print("Degree");
        Serial.println(degree);
#endif
        DistanceHandler();
        MessageGenerator();
        RFWriter(false);

        if (preYaw < -90 && nowYaw > 90)
        {
            if (degree > 0)
            {
                preYaw = 360 + preYaw;
            }
            else if (degree < 0)
            {
                nowYaw = nowYaw - 360;
            }
        }
        else if (nowYaw < -90 && preYaw > 90)
        {
            if (degree > 0)
            {
                nowYaw = 360 + nowYaw;
            }
            else if (degree < 0)
            {
                preYaw = preYaw - 360;
            }
        }

        if (preYaw > nowYaw)
        {
            if (degree <= preYaw && degree >= nowYaw)
            {
                break;
            }
        }
        else if (preYaw < nowYaw)
        {
            if (degree >= preYaw && degree <= nowYaw)
            {
                break;
            }
        }
    }
}

inline void RFWriter(bool startListening)
{
    rf24.stopListening();
    rf24.write(messageTransmit, sizeof(messageTransmit));
    if (startListening)
    {
        rf24.startListening();
    }
}

inline void RFReader()
{
    if (rf24.available())
    {
        rf24.read(messageRecieve, sizeof(messageRecieve));
        mode = messageRecieve[MSG_R_INDEX_MODE];
        leftSpeed = messageRecieve[MSG_R_INDEX_LSPEED];
        rightSpeed = messageRecieve[MSG_R_INDEX_RSPEED];
        MotorMove(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
        lostCounter = 0;
        digitalWrite(PIN_LED_SIG, LOW);
#ifdef DEBUG
        Serial.println("Connected");
#endif
    }
    else
    {
        ++lostCounter;
#ifdef DEBUG
        Serial.println("Lost!!!!!");
#endif
    }
    if (lostCounter >= SIG_LOST_TOLERANCE)
    {
        MotorStop();
        digitalWrite(PIN_LED_SIG, HIGH);
        lostCounter = SIG_LOST_TOLERANCE;
    }
}

inline void MPUHandler()
{
    if (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }
    }
    else
    {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        //get current FIFO count
        fifoCount = mpu.getFIFOCount();
        if (fifoCount < packetSize)
        {
            //Skip
        }
        // check for overflow (this should never happen unless our code is too inefficient)
        else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
        {
            mpu.resetFIFO(); // reset so we can continue cleanly
        }
        else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) // otherwise, check for DMP data ready interrupt (this should happen frequently)
        {
            // read a packet from FIFO
            while (fifoCount >= packetSize)
            { // Lets catch up to NOW, someone is using the dreaded delay()!
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
            }
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yprDegree[YPR_Y] = (ypr[0] * 180 / M_PI);
            yprDegree[YPR_P] = (ypr[1] * 180 / M_PI);
            yprDegree[YPR_R] = (ypr[2] * 180 / M_PI);
        }
    }
}

inline void dmpDataReady()
{
    mpuInterrupt = true;
}

inline void DistanceHandler()
{
    ReadEncoder();
    if ((leftSpeed >= 0 && rightSpeed <= 0) || (leftSpeed <= 0 && rightSpeed >= 0))
    {
        //Skip
    }
    else
    {
        distance += (((abs((signed long)(encoder1 - encoderp1)) + abs((signed long)(encoder2 - encoderp2))) / (double)2) * 0.1143) * cos((float)abs(ypr[YPR_P]));
        if (distance > 60000)
        {
            distance = 0;
        }
    }

#ifdef DEBUG
    Serial.print(distance);
    Serial.print("\t");
    Serial.print(encoder1);
    Serial.print("\t");
    Serial.print(encoderp1);
    Serial.print("\t");
    Serial.print(encoder2);
    Serial.print("\t");
    Serial.print(encoderp2);
    Serial.print("\t");
    Serial.print((((abs((signed long)(encoder1 - encoderp1)) + abs((signed long)(encoder2 - encoderp2))) / (double)2) * 0.1143) * cos((float)abs(ypr[YPR_P])));
    Serial.print("\t");
    Serial.println(distance);
#endif
}

inline void MessageGenerator()
{
    messageTransmit[MSG_T_INDEX_ST_SLOPE] = (((unsigned char)state) << 7) | ((unsigned char)((abs(yprDegree[YPR_P]) > 90) ? (180 - abs(yprDegree[YPR_P])) : (abs(yprDegree[YPR_P]))));
    messageTransmit[MSG_T_INDEX_DISTANCE1] = (unsigned char)((unsigned int)distance >> 8);
    messageTransmit[MSG_T_INDEX_DISTANCE2] = (unsigned char)((unsigned int)distance & 0xFF);
#ifdef DEBUG
    Serial.println((int)((abs(yprDegree[YPR_P]) > 90) ? (180 - abs(yprDegree[YPR_P])) : (abs(yprDegree[YPR_P]))));
#endif
}

inline void MotorMove(char motor1, char motor2, char motor3, char motor4)
{
    if (motor1 == 0 && motor2 == 0 && motor3 == 0 && motor4 == 0)
    {
        MotorStop();
    }
    else
    {
        Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
        Wire.write("ba");
        Wire.write((motor1 > 0) ? "f" : "r");
        Wire.write((motor2 > 0) ? "r" : "f");
        Wire.write((motor3 > 0) ? "r" : "f");
        Wire.write((motor4 > 0) ? "f" : "r");
        Wire.write(abs(motor1));
        Wire.write(0);
        Wire.write(abs(motor2));
        Wire.write(0);
        Wire.write(abs(motor3));
        Wire.write(0);
        Wire.write(abs(motor4));
        Wire.write(0);
        Wire.endTransmission();
        state = true;
    }
}

inline void MotorStop()
{
    Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
    Wire.write("ha");
    Wire.endTransmission();
    state = false;
}

inline void ReadEncoder() //true == Right  //false == Left
{
    encoderp1 = encoder1;
    encoderp2 = encoder2;
    Wire.beginTransmission(COM_BASEBOARD_ADDRESS);
    Wire.write("i");
    Wire.endTransmission();
    Wire.requestFrom(COM_BASEBOARD_ADDRESS, 8);
    while (Wire.available() == 8)
    {
        encoder1 = (unsigned long)Wire.read();
        encoder1 += ((unsigned long)Wire.read() << 8);
        encoder1 += ((unsigned long)Wire.read() << 16);
        encoder1 += ((unsigned long)Wire.read() << 24);
        encoder2 = (unsigned long)Wire.read();
        encoder2 += ((unsigned long)Wire.read() << 8);
        encoder2 += ((unsigned long)Wire.read() << 16);
        encoder2 += ((unsigned long)Wire.read() << 24);
    }
}
