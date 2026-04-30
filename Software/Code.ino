#include <Arduino.h>
#include <QTRSensors.h>

//Pines de motores y calibración
#define BOTTOM 12
#define LED_CAL 2
#define PinA1 10
#define PinB1 5
#define PinA2 11
#define PinB2 6

//Placa de sensores
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues [SensorCount];

// POSICIÓN 
int pos = 0;
int errorLast = 0;
int sumap = 0;
int suma = 0;

// PID (REVISAR) 
float kp = 0.7;
float kd = 4.0;
float setpoint = 2500;
int correccion = 0;
int baseSpeed = 130;

//Control motores
class Motor {
    private:
    int PinA;
    int PinB;

    public:
    Motor (int _PinA,int _PinB): PinA(_PinA),PinB(_PinB) {}
    void begin(){
        pinMode (PinA, OUTPUT);
        pinMode (PinB, OUTPUT);
        digitalWrite (PinA, HIGH);
        digitalWrite (PinA, HIGH);
    }
    void setSpeed(int speed){
        speed = constrain(speed, -255, 255);
        if (speed > 0){ 
        digitalWrite(PinA, HIGH);
        analogWrite (PinB, 255 -speed);
        } else if (speed < 0){
            analogWrite (PinA, 255 +speed);
            digitalWrite(PinB, HIGH);
        } else {
            digitalWrite (PinA, HIGH);
            digitalWrite (PinB, HIGH);
        }
    }
} ;

Motor M1(PinA1, PinB1);
Motor M2(PinA2, PinB2);

void setup(){
  Serial.begin(115200);
  pinMode(BOTTOM, INPUT_PULLUP);
  pinMode(LED_CAL, OUTPUT);

  M1.begin();   // Iniciar motor 1
  M2.begin();   // Inicia motor 2
  
  qtr.setTypeAnalog();
  uint8_t sensorPins[]={A0,A1,A2,A3,A4,A5,A6,A7};
  qtr.setSensorPins(sensorPins, SensorCount);
  digitalWrite (LED_CAL, LOW);
  while(digitalRead(BOTTOM)){}
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_CAL, HIGH);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t pos = qtr.readLineBlack(sensorValues);
   //PD
  float error    = (float)pos - setpoint;
  float derivada = error - errorLast;
  correccion     = (error * kp) + (derivada * kd);
  errorLast      = error;

  M1.setSpeed(constrain(baseSpeed - correccion, -255, 255));
  M2.setSpeed(constrain(baseSpeed + correccion, -255, 255));

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(pos);

  delay(250);
}
