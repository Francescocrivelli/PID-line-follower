#include <OpenTB6612FNG.h>
#include <Lamborghino.h>

#define BOTON  12
#define BUZZER  10

int setpoint = 0;
int gyroSpeed = 80;
int base = 40;


float Kprop = 1.1;
float Kderiv = 0.9;
float Kinte = 0.1;

Lamborghino Lamborghino(BOTON, BUZZER);
OpenTB6612FNG OpenTB6612FNG(0);

void setup() {
  Serial.begin(9600);
  Lamborghino.WaitBoton();
  Lamborghino.calibracion();
  Lamborghino.WaitBoton();
  delay(1000);
}
void loop() {

  Serialin();
  int pos =  Lamborghino.whiteLine();
  Lamborghino.PIDLambo(Kprop, Kderiv, Kinte);
  int Power = Lamborghino.PID(pos, setpoint, gyroSpeed);
  OpenTB6612FNG.Motores(base - Power, base + Power );

}


void Serialin() {

  if (Serial.available() > 0) {

    tone(BUZZER, 1300, 100);
    base = Serial.parseInt();
    Kprop = Serial.parseInt() / 10.0;
    Kderiv = Serial.parseInt() / 10.0;
    Kinte = Serial.parseInt() / 10.0;
    setpoint = Serial.parseInt();

    if (Serial.readString() == ('\n')) {
      Serial.flush();
    }
  }
}
