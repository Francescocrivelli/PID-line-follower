 # include "Arduino.h"
 # include "Lamborghino.h"
 # include "OpenTB6612FNG.h"
 # include "QTRSensors.h"

 # define NUM_SENSORS 6 // number of sensors used
 # define NUM_SAMPLES_PER_SENSOR 4 // average 4 analog samples per sensor reading
 # define EMITTER_PIN 11 // emitter is controlled by digital


 # define Drueda 25
 # define HIZ A7 // pin Hito Izquierda
 # define HDE A6 // pin Hito Derecha


int umbral = 650;
int fin = 0;

int BOTON;
int BUZZER;

long ppr = 60.0;

long PasosIz = 0;
long PasosDe = 0;

long lPasosIz = 0;
long lPasosDe = 0;

long n = 0;
long m = 0;

long piz = 0;
long pde = 0;

long timeNow = 0;
long timeBefore = 0;
long sampletime = 15000; //microsegundos

long StimeNow = 0;
long StimeBefore = 0;
long Ssampletime = 100000; //microsegundos

long FBtimeNow = 0;
long FBtimeBefore = 0;
long FBsampletime = 15000; //microsegundos


float velrealiz = 0; //  velrealiz = (d2-d1)/(timeNow-timeBefore)
float velrealde = 0; //  velrealiz = (d2-d1)/(timeNow-timeBefore)
float velprom = 0;

int setv = 100;
int PowIz = 0;
int PowDe = 0;
int Pow = 0;

//--------------------PIDLambo--------------------------------------------
double position;
int derivative = 0; // Derivative
int proportional = 0; // proportional
int power_difference = 0; // Differential (velocity)
int last_proportional;
int RANGEBRAKE = 2500;

int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

float KP;
float KD;
float KI;

//--------------------Intersections--------------------------

int contHito = 0;
int geo = 0;
int geo1 = 0;
int geo2 = 0;
int geo3 = 0;
int geo4 = 0;
int geo5 = 0;

int Hiz = 0;
int Hde = 0;

//---------------map----------------

int x = 0;

int program = 1;

//Variables directas (obtenidas)
int mapade[50]; //pasos encoder
int mapaiz[50]; //pasos encoder
int mapa[50];
int rmapa[50];
int velocidades[50];
int mapcontador = 0;

//----------------- Velocities

int vrecta = 1000;
int vcurva = 500;

QTRSensorsAnalog qtra((unsigned char[]) {
	0,
	1,
	2,
	3,
	4,
	5
},
	NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

Lamborghino::Lamborghino(int PINBOTON, int PINBUZZER) {
	BUZZER = PINBUZZER;
	BOTON = PINBOTON;
	pinMode(PINBOTON, INPUT);
	pinMode(HIZ, INPUT);
	pinMode(HDE, INPUT);
	pinMode(PINBUZZER, OUTPUT);

}

void Lamborghino::WaitBoton() {   // began an infinite blücle of waiting
	while (!digitalRead(BOTON));  // bucle ends when the button os clicked
	tone(BUZZER, 2000, 100);      // once it's out of the bucle the buzzer start beeping
}

void Lamborghino::beep() {
	tone(BUZZER, 2000, 100);  // Bzzer sound from 100ms to 2000ms
}

void Lamborghino::IfBoton() {
	if (digitalRead(BOTON) == HIGH) 	{
		tone(BUZZER, 1000, 50);
		delay(200);
		WaitBoton();
		delay(200);
	}
}



void Lamborghino::calibracion() {

	tone(BUZZER, 1000, 100);
	for (int i = 0; i < 100; i++) // make the calibration take about 10 seconds
	{
		qtra.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
	}

	Serial.begin(9600);
	for (int i = 0; i < NUM_SENSORS; i++) {
		Serial.print(qtra.calibratedMinimumOn[i]);
		Serial.print(' ');
	}
	Serial.println();

	for (int i = 0; i < NUM_SENSORS; i++) {
		Serial.print(qtra.calibratedMaximumOn[i]);
		Serial.print(' ');
	}
	Serial.println();
	Serial.println();
	tone(BUZZER, 1500, 50);
	delay(70);
	tone(BUZZER, 1500, 50);
}

long Lamborghino::LineaNegra() {
	int posicion = qtra.readLine(sensorValues, true, false);
	posicion = map(posicion, 0, 5000, -255, 255);
	return posicion;
}

long Lamborghino::LineaBlanca() {
	int posicion = qtra.readLine(sensorValues, true, true);
	posicion = map(posicion, 0, 5000, -255, 255);
	return posicion;
}

void Lamborghino::EncoderIz() {
	PasosIz++;
	piz++;
}

void Lamborghino::EncoderDe() {
	PasosDe++;
	pde++;
}

void Lamborghino::vel() {

	timeNow = micros();
	if ((timeNow - timeBefore) > (sampletime)) {
		velrealiz = ((Drueda * 3.14) / ppr) * (1000000.0 / sampletime) * (PasosIz - n);
		velrealde = ((Drueda * 3.14) / ppr) * (1000000.0 / sampletime) * (PasosDe - m);
		velprom = (velrealiz + velrealde) / 2;
		timeBefore = timeNow;
		controlador();
		n = PasosIz;
		m = PasosDe;
	}
}

void Lamborghino::incomingByte(int a) {
	velocid = a;
}

int Lamborghino::controlador() {
	FBtimeNow = micros();
	if ((FBtimeNow - FBtimeBefore) > (FBsampletime)) {
		if (velocid == 0) {
			Pow = 0;
		}
		if (velprom < velocid) {
			Pow++;
			if (pow > 255) {
				pow = 255;
			}
		} else {
			Pow--;
			if (pow < 0) {
				pow = 0;
			}
		}
		FBtimeBefore = FBtimeNow;
	}
	return Pow;
}

void Lamborghino::comunicacion() {
	StimeNow = micros();
	if ((StimeNow - StimeBefore) > (Ssampletime)) {
		Serial.println(velprom);
		StimeBefore = StimeNow;
	}
}

void Lamborghino::PIDLambo(float kp, float kd, float ki) {

	KP = kp;
	KD = kd;
	KI = ki;
}

long Lamborghino::PID(int POS, int setpoint, int lim) {

	proportional = int(POS) - setpoint;
	derivative = proportional - last_proportional;
	last_proportional = proportional;
	int power_difference = (proportional * KP) + (derivative * KD);
	if (power_difference > lim)
		power_difference = lim;
	else if (power_difference < -lim)
		power_difference = -lim;
	return power_difference;
}

void Lamborghino::ReadSensors() {
	Hiz = analogRead(HIZ);
	Hde = analogRead(HDE);

	if (Hiz < umbral) {
		Hiz = 1;
	} else {
		Hiz = 0;
	}

	if (Hde < umbral) {
		Hde = 1;
	} else {
		Hde = 0;
	}
}

void Lamborghino::detecGeo() {

	ReadSensors();

	if (Hiz == 0 && Hde == 0) {
		geo = 0;
	}
	if (Hiz == 1 && Hde == 0) {
		geo = 1;
	}
	if (Hiz == 0 && Hde == 1) {
		geo = 2;
	}
	if (Hiz == 1 && Hde == 1) {
		geo = 3;
	}

	if (geo1 != geo) {
		if (geo == 0 && geo1 == 1 && geo2 == 0) {
			fin++;

			funcionHitoDe();

		}

		if (geo == 0 && geo1 == 2 && geo2 == 0) {

			funcionHitoIz();
		}

		if (geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {

			funcionCruce();


		}
		geo5 = geo4;
		geo4 = geo3;
		geo3 = geo2;
		geo2 = geo1;
		geo1 = geo;
	}
}

int Lamborghino::change() {
	int changer = program;
	return changer;
}

void Lamborghino::velocimetro() {
	velocid = velocidades[mapcontador];
}


void Lamborghino::funcionCruce() {
	Serial.println("Intersection");
}


void Lamborghino::funcionHitoIz() {
tone(BUZZER, 1500, 50);

			mapaiz[mapcontador] = int(0.335 * (piz));
			mapade[mapcontador] = int(0.335 * (pde));
			mapa[mapcontador] = int(0.5 * (mapaiz[mapcontador] + mapade[mapcontador]));
			rmapa[mapcontador] = abs(int(6 * (mapaiz[mapcontador] + mapade[mapcontador]) / ((mapaiz[mapcontador] - mapade[mapcontador]))));

			piz = 0;
			pde = 0;

			/*
			Serial.print(velocid);
			Serial.print("\t");
			Serial.print(mapaiz[mapcontador]);
			Serial.print("\t");
			Serial.print(mapade[mapcontador]);
			Serial.print("\t");
			Serial.println("Left Marker");
			 */
			mapcontador++;
}


void Lamborghino::funcionHitoDe() {

			tone(BUZZER, 2000, 50);

			mapaiz[mapcontador] = int(0.335 * (piz));
			mapade[mapcontador] = int(0.335 * (pde));
			mapa[mapcontador] = int(0.5 * (mapaiz[mapcontador] + mapade[mapcontador]));
			rmapa[mapcontador] = abs(int(6 * (mapaiz[mapcontador] + mapade[mapcontador]) / ((mapaiz[mapcontador] - mapade[mapcontador]))));

			piz = 0;
			pde = 0;
			
			/*
			Serial.print(velocid);
			Serial.print("\t");
			Serial.print(mapaiz[mapcontador]);
			Serial.print("\t");
			Serial.print(mapade[mapcontador]);
			Serial.print("\t");
			Serial.println("Right Marker");
			 */
			 
			mapcontador++;

			
			if (fin >= 2) {
				velocid = 0;
				program++;
				Serial.println(program);
				mapcontador = 0;
				fin = 0;
			}
			
			
}

void Lamborghino::datalogger() {

	Serial.print("Mapiz:   ");
	for (int i = 0; i <= 255; i++) {
		if (mapaiz[i] == 0) {
			break;
		}
		Serial.print(mapaiz[i]);
		Serial.print("\t");
	}

	Serial.println();
	Serial.print("Mapde:   ");

	for (int i = 0; i <= 255; i++) {
		if (mapaiz[i] == 0) {
			break;
		}
		Serial.print(mapade[i]);
		Serial.print("\t");
	}

	Serial.println();
	Serial.print("Mapa:   ");

	for (int i = 0; i <= 255; i++) {
		if (mapaiz[i] == 0) {
			break;
		}
		Serial.print(mapa[i]);
		Serial.print("\t");
	}

	Serial.println();
	Serial.print("Rmapa:   ");

	for (int i = 0; i <= 255; i++) {
		if (mapaiz[i] == 0) {
			break;
		}
		Serial.print(rmapa[i]);
		Serial.print("\t");
	}
	Serial.println("\t");
}
