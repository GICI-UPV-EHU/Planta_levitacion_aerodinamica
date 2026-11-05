#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// El Pin 9 está ligado al canal A del Timer 1 (OC1A)
const int pinPWM = 9;

// --- CONFIGURACIÓN DEL CONTROLADOR PROPORCIONAL ---
float consigna = 200.0;     // Consigna inicial en milímetros (puedes cambiarla por el Serial)
float Kp = 0.03;             // Ganancia Proporcional (ajusta este valor según la respuesta de tu sistema)
float porcentajePWM = 93; // Empezamos en el punto de operación (95%)

// Temporización no bloqueante
unsigned long tiempoAnterior = 0;
const long intervaloControl = 100; // El bucle de control se ejecuta cada 100 ms

uint16_t distanciaActual = 0;

void setup() {
  Serial.begin(9600);
  
  delay(500); 
  Wire.begin();

  if (!sensor.init()) {
    Serial.println("Error_Inicializacion_VL53L0X");
    while (1);
  }

  sensor.setTimeout(1000);
  sensor.setSignalRateLimit(0.1);
  sensor.setMeasurementTimingBudget(50000); // Reducido a 50ms para un control más rápido

  pinMode(pinPWM, OUTPUT);
  
  // --- CONFIGURACIÓN DEL TIMER 1 PARA 25 kHz ---
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1A |= (1 << COM1A1);          
  TCCR1B |= (1 << WGM13) | (1 << CS10); 
  ICR1 = 320; 
  
  // Fijar el punto de operación inicial (95% de 320 = 304)
  OCR1A = 304; 
  
  // Cabecera para el Serial Plotter
  Serial.println("Consigna_mm,Distancia_mm,Duty_Cycle_Porcentaje");
}

void loop() {
  // 1. Leer los comandos del Monitor Serie (Entrada de la nueva consigna)
  leerConsignaSerial();

  // 2. Subprograma de Control Realimentado (Temporizado)
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervaloControl) {
    tiempoAnterior = tiempoActual;
    
    leerSensor();
    ejecutarControlProporcional();
    enviarDatosPlotter();
  }
}

// ==========================================
// SUBPROGRAMA 1: Lectura del Sensor Láser
// ==========================================
void leerSensor() {
  uint16_t lectura = sensor.readRangeSingleMillimeters();

  if (sensor.timeoutOccurred() || lectura > 2000) {
    distanciaActual = 2000; // Acotación por fuera de rango
  } else {
    distanciaActual = lectura;
  }
}

// ==========================================
// SUBPROGRAMA 2: Lazo Cerrado y Escritura PWM
// ==========================================
void ejecutarControlProporcional() {
  // Calcular el error del sistema
  float error = consigna - distanciaActual;
  
  // Ecuación de control P con punto de operación al 95%
  porcentajePWM = 92.5 - (Kp * error);
  
  // --- SATURACIÓN ESTRICTA (Restringido entre 90% y 100%) ---
  if (porcentajePWM > 100.0) porcentajePWM = 100.0;
  if (porcentajePWM < 86.0)  porcentajePWM = 86.0;
  
  // Convertir el porcentaje final (90%-100%) al rango del Timer 1 (0 a 320)
  // El mapeo flotante manual asegura no perder resolución
  int valorOCR1A = (porcentajePWM / 100.0) * 320;
  
  // Aplicar acción de control al registro físico
  OCR1A = valorOCR1A;
}

// ==========================================
// FUNCIÓN: Captura de nueva consigna por Serial
// ==========================================
void leerConsignaSerial() {
  if (Serial.available() > 0) {
    // Lee el número entero o flotante introducido en el monitor serie
    float nuevaConsigna = Serial.parseFloat();
    
    // Validar que sea una distancia lógica antes de aplicarla
    if (nuevaConsigna > 0 && nuevaConsigna <= 2000) {
      consigna = nuevaConsigna;
    }
  }
}

// ==========================================
// FUNCIÓN AUXILIAR: Graficación en Serial Plotter
// ==========================================
void enviarDatosPlotter() {
  // Enviamos tres variables: la referencia, la medida real y la acción de control
  Serial.print(consigna);
  Serial.print(",");
  Serial.print(distanciaActual);
  Serial.print(",");
  Serial.println(porcentajePWM);
}