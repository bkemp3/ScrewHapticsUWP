/* --- MOTOR LIBRARIES --- */
#include <Stepper.h>

/* --- BLUETOOTH LIBRARIES --- */
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/* --- IMU LIBRARIES --- */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/* --- --- DEFINE GLOBAL VARIABLES --- --- */

/* --- ENCODER VARIABLES --- */
#define encoderPinA 1 //outputA digital pin2
#define encoderPinB 2 //outoutB digital pin3
volatile int count = 0; // raw encoder count 
int protectedCount = 0; // readable encoder count
int previousCount = 0; // previous encoder count
int prev_a, prev_b = 1; // previous encoder values
const float countsPerRevolution = 2400; // number of counts per revolution for encoder
float revolutions = 0.0f; 
float stepperPos;
float desiredPos; 

/* --- STEPPER MOTOR VARIABLES --- */
#define motorPin1 3
#define motorPin2 4
#define motorPin3 5
#define motorPin4 6
#define LIMIT_SWITCH_PIN 7
const int stepsPerRevolution = 200;  // number of steps per revolution for your motor
bool limit; 
enum States {calibrate, force};
States State = calibrate; 
// initialize the stepper library on 4 digital pins:
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

/* --- BLE VARIABLES --- */
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
std::string dataIn = "pleaseWork";


/* --- IMU VARIABLES --- */
#define INTERRUPT_PIN 21
#define IMU_SDA_PIN 10
#define IMU_SCL_PIN 42
#define OUTPUT_READABLE_QUATERNION
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;

/* --- LINEAR ACTUATOR VARIABLES --- */
#define LA1_ENABLE_PIN -1
#define LA1_CONTROL_PIN1 -1
#define LA1_CONTROL_PIN2 -1
#define LA2_ENABLE_PIN -1
#define LA2_CONTROL_PIN1 -1
#define LA2_CONTROL_PIN2 -1
int LA_position = 0;

/* --- --- CALLBACKS --- --- */
void dmpDataReady() {
    mpuInterrupt = true;
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class writeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      dataIn = pCharacteristic->getValue();
      // Serial.println(String(dataIn.c_str()));
    }
};


void isrA() {
  int a;
  if (prev_a==1) {a = 0;} 
  else {a = 1;}
  if(a==prev_b) {count ++;} 
  else {count --;}
  prev_a = a;
}
void isrB() {
  int b;
  if (prev_b==1) {b = 0;} 
  else {b = 1;}
  if(b==prev_a) {count --;} 
  else {count ++;}
  prev_b = b;
}

/* --- SETUP FUNCTIONS --- */
void setup() {
  // Serial.begin(115200);
  ENCsetup(); // set up encoder
  STEPsetup(); // set up stepper motor TODO: implement me
  BTsetup(); // set up bluetooth
  IMUsetup(); // set up IMU
  //LAsetup(); // set up linear actuator
}

void ENCsetup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE);

}

void STEPsetup() {
  myStepper.setSpeed(80);

  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

}

void BTsetup() {
    // Create the BLE Device
  BLEDevice::init("ESP32-RUST");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->setCallbacks(new writeCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  // Serial.println("Waiting a client connection to notify...");

}

void IMUsetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(IMU_SDA_PIN,IMU_SCL_PIN); // SDA, SCL
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }

}

void LAsetup() { 
  pinMode(LA1_ENABLE_PIN, OUTPUT);
  pinMode(LA1_CONTROL_PIN1, OUTPUT);
  pinMode(LA1_CONTROL_PIN2, OUTPUT);
  pinMode(LA2_ENABLE_PIN, OUTPUT);
  pinMode(LA2_CONTROL_PIN1, OUTPUT);
  pinMode(LA2_CONTROL_PIN2, OUTPUT);
  LA_reset_position(); // make sure LA starts at base position

}

/* --- LOOP FUNCTIONS --- */
void loop() {
  ENCloop(); // loop encoder
  STEPloop(); // loop stepper motor TODO: implement me
  BTloop(); // loop bluetooth
  IMUloop(); // loop IMU
  //LAloop(); // loop linear actuator TODO: implement me

}

void ENCloop() { // MAKE SURE ALL OTHER FNCS USE PROTECTED COUNT
  noInterrupts();
  protectedCount = count;
  interrupts();
  previousCount = protectedCount;
  // Serial.println(previousCount);
  updateRev();
}

void STEPloop() {
  switch (State) {
    case calibrate:
      // Serial.println("calibrate");

      myStepper.step(200);
      delay(2000);
      limit = digitalRead(LIMIT_SWITCH_PIN);
      while (limit == HIGH) {
        limit = digitalRead(LIMIT_SWITCH_PIN);
        if (limit == LOW) {
          stepperPos = 0.0; 
          break; // This break exits the while loop
        }
        myStepper.step(-1);
        delay(50);
      }

      State = force; // Change state after exiting the while loop
      moveToPosition(1);
      break; // This break exits the case

    case force:
      // updateRev();

      // Serial.println(revolutions);
      if (revolutions >= 0.25 && revolutions <= 1) {
        moveToPosition(2.7);
      } else if (revolutions > 1 && revolutions <= 1.5) {
        moveToPosition(2);
      } else if (revolutions > 1.5 && revolutions <= 2) {
        moveToPosition(1.5);
      }


      break; // Add break here to exit the case
  }
}

void BTloop() {
  // notify changed value
    if (deviceConnected) {
        // pCharacteristic->setValue((unsigned char*)value, 4);
        String toSend = String(q.x) + "," + String(q.y) + "," + String(q.z) + "," + String(q.w) + "," + String(revolutions) + "," + "Hehe";
        // Serial.println(toSend);
        // String toSend = String(1.0f);
        pCharacteristic->setValue(toSend.c_str());
        pCharacteristic->notify();    
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        // Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    //for unity
    delay(30);
}

void IMUloop() {
  //get world acceleration
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
          //  Serial.print("quat\t");

            // Serial.print(q.w);
            // Serial.print(",");
            // Serial.print(q.x);
            // Serial.print(",");
            // Serial.print(q.y);
            // Serial.print(",");
            // Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);//
            mpu.dmpGetEuler(euler, &q);
            // Serial.print("euler\t");
            // Serial.print(euler[0] * 180/M_PI);
            // Serial.print(",");
            // Serial.print(euler[1] * 180/M_PI);
            // Serial.print(",");
            // Serial.println(euler[2] * 180/M_PI);
        #endif
    }

}

void LAloop() {

}

/* --- HELPER FUNCTIONS --- */

/* --- STEPPER HELPER FUNCTIONS --- */
void updateRev(){
  revolutions = (float) protectedCount / (float) countsPerRevolution;
}

void moveToPosition(float desiredPos){ //desired linear pos in mm 
  //moves to absolute on linear rail (50mm stroke) by calculating 
  // updateRev();
  int desiredPosSteps = round(desiredPos * (stepsPerRevolution/10.0));
  //Serial.println(desiredPosSteps);
  float relativeMove = (desiredPosSteps - stepperPos);
  myStepper.step(relativeMove);
  float oldStepperPos = stepperPos; 
  stepperPos = oldStepperPos + relativeMove;
  
}

/* --- LINEAR ACTUATOR HELPER FUNCTIONS --- */
void LA_reset_position() {
  LA_backwards(30);
  LA_position = 0;

}

void LA_backwards(int millis) {
  // set backwards polarity
  digitalWrite(LA1_CONTROL_PIN1, LOW);
  digitalWrite(LA1_CONTROL_PIN2, HIGH);
  digitalWrite(LA2_CONTROL_PIN1, LOW);
  digitalWrite(LA2_CONTROL_PIN2, HIGH);

  // move motor backwards
  digitalWrite(LA1_ENABLE_PIN, HIGH);
  digitalWrite(LA2_ENABLE_PIN, HIGH);
  delay(millis*100);
  digitalWrite(LA1_ENABLE_PIN, LOW);
  digitalWrite(LA2_ENABLE_PIN, LOW);

  LA_position -= millis;

}

void LA_forwards(int millis) {
  // set backwards polarity
  digitalWrite(LA1_CONTROL_PIN1, HIGH);
  digitalWrite(LA1_CONTROL_PIN2, LOW);
  digitalWrite(LA2_CONTROL_PIN1, HIGH);
  digitalWrite(LA2_CONTROL_PIN2, LOW);

  // move motor backwards
  digitalWrite(LA1_ENABLE_PIN, HIGH);
  digitalWrite(LA2_ENABLE_PIN, HIGH);
  delay(millis*100);
  digitalWrite(LA1_ENABLE_PIN, LOW);
  digitalWrite(LA2_ENABLE_PIN, LOW);

  LA_position += millis;

}

