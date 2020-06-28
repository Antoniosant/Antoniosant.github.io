// Antonio Santiago Fernández
// Desarrollo de una interfaz para realidad virtaul movil 
// Grado de Sistemas Electrónicos
// Curso 2019-2020
//Código basado en el ejemplo MPU6050_DMP6 de la libreria libreria desarrollada por Jeff Rowberg

//El código consiste en extraer los angulos yaw, pitch y roll del sensor MPU6050 y segun
//sus angulos escribir una letra u otra. Tambien consta de botones, aunque ambos enciende dos leds
//distintos, el boton de la izquierda permite la escritura de teclas y el botón de la derecha, envia una 
//tecla.
 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Keyboard.h>
#include <VL53L0X.h>
 
#define INTERRUPT_PIN 2
#define LEDR_PIN 9
#define LEDG_PIN 10
#define PIN_BD 5
#define PIN_BI 6

MPU6050 mpu;
VL53L0X sensor;
 
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
Quaternion q;           // [w, x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

int distancia;
bool on =false;
bool estadoled = false;

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}
 
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 
    Serial.begin(9800);
 
    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
 
    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 
    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 
    // Valores de calibracion
    mpu.setXGyroOffset(261);
    mpu.setYGyroOffset(-59);
    mpu.setZGyroOffset(24);
    mpu.setZAccelOffset(1374);
 
    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
 
        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    //-------------------------------------
    //CONFIGURACION SENSOR TIEMPO DE VUELO
    //-------------------------------------
    sensor.setTimeout(500);
     if (!sensor.init())
     {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) {}
     }
    sensor.startContinuous();
    
    //--------------------------------------
    // CONFIGURACION BOTONES Y LEDS
    //--------------------------------------
    pinMode(PIN_BD,INPUT);
    pinMode(PIN_BI,INPUT);
    pinMode(LEDR_PIN, OUTPUT);
    pinMode(LEDG_PIN, OUTPUT);
    
}
 
// ================================================================
// ===                    PROGRAMA PRINCIPAL                    ===
// ================================================================
void loop() {
    // Si fallo al iniciar, parar programa
    if (!dmpReady) return;
 
    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
        // AQUI EL RESTO DEL CODIGO DE TU PROGRAMA
    }
 
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();
 
    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
   // Mostrar Yaw, Pitch, Roll
   mpu.dmpGetQuaternion(&q, fifoBuffer);
   mpu.dmpGetGravity(&gravity, &q);
   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
   Serial.print("ypr\t");
   Serial.print(ypr[0] * 180/M_PI);
   Serial.print("\t");
   Serial.print(ypr[1] * 180/M_PI);
   Serial.print("\t");
   Serial.println(ypr[2] * 180/M_PI);
 //Presiono boton izquierdo
 if(digitalRead(PIN_BI)==LOW){
      delay(50);
      if(digitalRead(PIN_BI)==LOW){  
          on = !on; 
          estadoled = !estadoled;
        } 
        if(estadoled){
            digitalWrite(LEDR_PIN,HIGH);
        } else {
            digitalWrite(LEDR_PIN,LOW);
        }
        if(on){
            Keyboard.press(' ');   
        } else {
            Keyboard.release(' ');
        }
 }
 //Presiono boton derecho
 if(digitalRead(PIN_BD)==LOW){
      delay(50);
      if(digitalRead(PIN_BD)==LOW){
          Keyboard.press('e');
          digitalWrite(LEDG_PIN,HIGH);
         
        } else {
          digitalWrite(LEDG_PIN,LOW);
          Keyboard.release('e');
       }
        
 }
 
  if(on){
   distancia = sensor.readRangeContinuousMillimeters()/10;
    if(distancia > 110){
      Keyboard.press('q'); 
    } else {
      Keyboard.release('q');
    }
       if((ypr[0] * 180/M_PI) > 20){
         Serial.println( "Atras");
          Keyboard.press('s');
       } else {
          Keyboard.release('s');
       } 
       if((ypr[2] * 180/M_PI)> 110){
            Serial.println( "izquierda");
            Keyboard.press('a');
       } else {
            Keyboard.releaseAll();
       }
       if((ypr[0] * 180/M_PI)< -20){
          Serial.println( "Adelante");
          Keyboard.press('w');
       } else {
          Keyboard.release('w');
       }
       if((ypr[2] * 180/M_PI) < 70){
          Serial.println( "derecha");
           Keyboard.press('d');
       } else {
           Keyboard.release('d');
       }
       
   
  }
    }
}
