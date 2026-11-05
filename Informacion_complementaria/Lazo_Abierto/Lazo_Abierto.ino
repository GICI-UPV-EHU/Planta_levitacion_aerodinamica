#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// El Pin 9 está ligado al canal A del Timer 1 (OC1A)
const int pinPWM = 9;

// Variable para el Duty Cycle (rango de 0 a 320 para 25kHz)
int valorPWM = 0; 

// Pasos de incremento/decremento
const int pasoGrueso = 16; // 5% (16 unidades)
const int pasoFino = 2;    // Ajuste de precisión unidad por unidad

// --- CONFIGURACIÓN DE LA CALIBRACIÓN (INTERPOLACIÓN) ---
const int NUM_PUNTOS = 7;
// Puntos de control medidos (deben estar estrictamente ordenados de menor a mayor)
const float puntosCalibracion[NUM_PUNTOS] = {0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0};
// Errores medidos en cada uno de esos puntos
const float erroresMedidos[NUM_PUNTOS]    = {0.0, 40.0, 60.0, 80.0, 85.0, 80.0, 40.0};

void setup() {
  Serial.begin(9600);
  
  // Retraso para estabilizar el sensor eléctricamente
  delay(500); 
  
  Wire.begin();

  // Inicializamos el sensor
  if (!sensor.init()) {
    while (1); // Si falla el sensor, detiene la ejecución
  }

  // Configuración de tiempo de espera y precisión
  sensor.setTimeout(1000);
  sensor.setSignalRateLimit(0.1);
  sensor.setMeasurementTimingBudget(200000); // 200ms por lectura

  // Configurar el Pin 9 como salida
  pinMode(pinPWM, OUTPUT);
  
  // --- CONFIGURACIÓN DEL TIMER 1 PARA 25 kHz ---
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Modo 10: PWM con corrección de fase y frecuencia (TOP en ICR1)
  TCCR1A |= (1 << COM1A1); 
  TCCR1B |= (1 << WGM13) | (1 << CS10); // Prescaler = 1
  
  // TOP = 16,000,000 Hz / (2 * 1 * 25,000 Hz) = 320
  ICR1 = 320; 

  // Inicializar el ventilador apagado (0% Duty Cycle)
  OCR1A = 0; 
}

void loop() {
  // Leemos la distancia en modo "Single"
  uint16_t altura = 655.0 - sensor.readRangeSingleMillimeters();
  float error = calcularErrorInterpolado((float)altura);
  float altura_real = (float)altura + error;
  
  // Calculamos el porcentaje del Duty Cycle
  float porcentaje = (valorPWM / 320.0) * 100.0;

  // --- FORMATO PARA EXCEL (CSV: Columna A = Porcentaje, Columna B = Distancia) ---
  Serial.print(porcentaje, 1);
  Serial.print(","); // La coma separa las columnas en Excel
  
  if (sensor.timeoutOccurred() || altura > 800) {
    Serial.println("-1"); // Marca de error o fuera de rango (para graficar fácil en Excel)
  } else {
    Serial.println(altura_real); // Imprime distancia y salto de línea (nueva fila en Excel)
  }

  // --- LECTURA DE COMANDOS DESDE EXCEL O IDE ---
  if (Serial.available() > 0) {
    char primerCaracter = Serial.peek(); // Observamos el primer carácter
    bool cambio = false;

    // Si el carácter ingresado es un dígito, leemos el número completo
    if (isDigit(primerCaracter)) {
      int valorIngresado = Serial.parseInt();
      valorPWM = valorIngresado;
      cambio = true;
    } else {
      char comando = Serial.read(); // Consumimos el carácter de comando
      
      // Controles de paso grueso (+/- 16 -> 5%)
      if (comando == '+') {
        valorPWM += pasoGrueso;
        cambio = true;
      } 
      else if (comando == '-') {
        valorPWM -= pasoGrueso;
        cambio = true;
      }
      // Controles de paso fino (+/- 2)
      else if (comando == 'w' || comando == 'W') {
        valorPWM += pasoFino;
        cambio = true;
      }
      else if (comando == 's' || comando == 'S') {
        valorPWM -= pasoFino;
        cambio = true;
      }
    }

    if (cambio) {
      // Garantizar límites del registro ICR1 (0 a 320)
      if (valorPWM > 320) valorPWM = 320;
      if (valorPWM < 0) valorPWM = 0;

      // Aplicar el nuevo Duty Cycle directamente al registro del Timer 1
      OCR1A = valorPWM;
    }
  }

  delay(300); // Pausa entre muestras
}

// Interpolación Lineal para Cálculo del Error

float calcularErrorInterpolado(float distancia) {
  if (distancia <= puntosCalibracion[0]) {
    return erroresMedidos[0];
  }
  if (distancia >= puntosCalibracion[NUM_PUNTOS - 1]) {
    return erroresMedidos[NUM_PUNTOS - 1];
  }
  for (int i = 0; i < NUM_PUNTOS - 1; i++) {
    if (distancia >= puntosCalibracion[i] && distancia <= puntosCalibracion[i+1]) {
      float x0 = puntosCalibracion[i];
      float x1 = puntosCalibracion[i+1];
      float y0 = erroresMedidos[i];
      float y1 = erroresMedidos[i+1];
      
      return y0 + ((distancia - x0) * (y1 - y0) / (x1 - x0));
    }
  }
  return 0.0;
}