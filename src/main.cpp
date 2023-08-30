#include <Arduino.h>
#include "config.h"

AdafruitIO_Feed *tempCanal = io.feed("Temperatura");

const int Boton = 26;
const int sensor = 34;
const int Rpin = 25;
const int Gpin = 33;
const int bluePin = 32;
const int servoPin = 22;

// Pines para el display de 7 segmentos
const int displayPins[] = {4, 23, 12, 16, 14, 13, 15, 18};
const int digitPins[] = {5, 17, 19}; // Pines de control de dígitos

// Definición de segmentos para cada dígito del display de 7 segmentos
byte segmentos[] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

float temperatura = 0.0;
int Coloractual = -1;   // Color actual del LED (-1: apagado, 0: verde, 1: oro, 2: rojo)

// Declaración de funciones
void LEDactualizado(int color);
void Servoactualizado(int posicion);
void mostrarTemperaturaEnDisplay(float temp);

void setup() {
  Serial.begin(115200);

  pinMode(Boton, INPUT_PULLUP);
  pinMode(sensor, INPUT);
  pinMode(Rpin, OUTPUT);
  pinMode(Gpin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(servoPin, OUTPUT);

  // Configuración de pines de control de dígitos
  for (int i = 0; i < 3; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], HIGH); // Inicialmente apagados
  }

  // LED y el servo en la posición inicial
  LEDactualizado(-1);  // Inicialmente apagado
  Servoactualizado(90);  // Inicialmente a 90 grados

  // wait for serial monitor to open
  while (!Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}

void loop() {
  // io.run() is required for Adafruit IO communication
  io.run();

  if (digitalRead(Boton) == LOW) { // El botón es presionado
    int Sensorvalor = analogRead(sensor);
    float V = (Sensorvalor / 4095.0) * 3300.0; // ADC a mV

    V = V + 200.0; //Offset a la lectura para tener el valor ajustado de temperatura

    temperatura = V / 10.0; //Calibración del sensor

    Serial.print("V: ");
    Serial.print(V);
    Serial.print(" mV   temperatura: ");
    Serial.print(temperatura);
    Serial.println(" °C");

    int Colornuevo = -1; //Variable que inicia en apagado

    if (temperatura < 37.0) {
      Colornuevo = 0; // Verde
    } else if (temperatura < 37.5) {
      Colornuevo = 1; // Oro
    } else {
      Colornuevo = 2; // Rojo
    }

    if (Colornuevo != Coloractual) { //Se verifica si existió un cambio de color 
      Coloractual = Colornuevo;
      LEDactualizado(Coloractual);

      // Actualiza la posición del servo basada en el color actual
      if (Coloractual == 0) {
        Servoactualizado(160);
      } else if (Coloractual == 1) {
        Servoactualizado(90);
      } else if (Coloractual == 2) {
        Servoactualizado(20);
      }
    }

    Serial.print("sending -> ");
    Serial.println(temperatura);
    tempCanal->save(temperatura);

    // Mostrar la temperatura en el display de 7 segmentos
    mostrarTemperaturaEnDisplay(temperatura);

    delay(1000); // Delay para el botón
  }
}

void LEDactualizado(int color) {
  if (color == 0) {
    analogWrite(Rpin, 0);
    analogWrite(Gpin, 255);
    analogWrite(bluePin, 0);
  } else if (color == 1) {
    analogWrite(Rpin, 255);
    analogWrite(Gpin, 215);
    analogWrite(bluePin, 0);
  } else if (color == 2) {
    analogWrite(Rpin, 255);
    analogWrite(Gpin, 0);
    analogWrite(bluePin, 0);
  }
}

void Servoactualizado(int posicion) {
  analogWrite(servoPin, map(posicion, 0, 180, 0, 255));
}

void mostrarTemperaturaEnDisplay(float temp) {
  int temperaturaEntera = int(temp); // Parte entera de la temperatura
  int digitos[3]; // Dígitos individuales de la temperatura

  // Extraer dígitos individuales de la temperatura
  digitos[0] = temperaturaEntera % 10;
  digitos[1] = (temperaturaEntera / 10) % 10;
  digitos[2] = (temperaturaEntera / 100) % 10;

  // Mostrar dígitos en el display
  for (int i = 0; i < 3; i++) {
    // Encender el dígito correspondiente
    digitalWrite(digitPins[i], LOW);

    // Configurar los segmentos del display
    for (int j = 0; j < 7; j++) {
      digitalWrite(displayPins[j], (segmentos[digitos[i]] >> j) & 1);
    }

    // Esperar un breve periodo de tiempo para mostrar el dígito
    delay(1);

    // Apagar el dígito
    digitalWrite(digitPins[i], HIGH);
  }
}
