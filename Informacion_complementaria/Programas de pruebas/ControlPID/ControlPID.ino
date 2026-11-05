#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// El Pin 9 está ligado al canal A del Timer 1 (OC1A)
const int pinPWM = 9;

// --- CONFIGURACIÓN DEL CONTROLADOR PID ---
float consigna = 600.0;     // Consigna inicial en milímetros
float porcentajePWM = 0.0; // Punto de operación nominal

// Ganancias del PID (Ajustar según la dinámica de tu planta)
float Kp = 0.05;            // Ganancia Proporcional
float Ki = 0.01;            // Ganancia Integral
float Kd = 0.02;            // Ganancia Derivativa
float BIAS = 73.00;

// Variables internas del PID
float errorAnterior = 0.0;
float integral = 0.0;

// Temporización estricta para el cálculo del PID (dt constante)
unsigned long tiempoAnterior = 0;
const float dt = 0.1;       // Período de muestreo en segundos (100 ms)
const long intervaloControl = 100; // 100 ms en milisegundos

// --- NUEVAS VARIABLES PARA EL BUFFER CIRCULAR ---
const int TAMANO_BUFFER = 1;         // Tamaño del filtro (8 muestras va genial para 100ms)
float buffer[TAMANO_BUFFER];        // Array para almacenar las lecturas
int indiceBuffer = 0;               // Índice de escritura en el buffer
float sumaAcumulada = 0.0;          // Suma optimizada para evitar bucles for
bool bufferLleno = false;           // Bandera de inicialización del buffer
float distanciaFiltrada = 0.0;

// --- CONFIGURACIÓN DE LA CALIBRACIÓN (INTERPOLACIÓN) ---
const int NUM_PUNTOS = 6;
// Puntos de control medidos (deben estar estrictamente ordenados de menor a mayor)
const float puntosCalibracion[NUM_PUNTOS] = {140.0, 253.0, 358.0, 445.0, 531.0, 615.0};
// Errores medidos en cada uno de esos puntos (REEMPLAZA ESTOS NÚMEROS POR TUS MEDICIONES REALES)
const float erroresMedidos[NUM_PUNTOS]    = {40.0, 53.0, 58.0,  45.0, 31.0, 15.0};

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
  sensor.setMeasurementTimingBudget(50000); // 50ms de lectura para no interferir con los 100ms de control

  pinMode(pinPWM, OUTPUT);
  
  // --- CONFIGURACIÓN DEL TIMER 1 PARA 25 kHz ---
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  TCCR1A |= (1 << COM1A1);          
  TCCR1B |= (1 << WGM13) | (1 << CS10); 
  ICR1 = 320; 
  
  // Fijar el punto de operación inicial (78% de 320 = ~250)
  OCR1A = 250; 
  
  // --- INICIALIZACIÓN DEL BUFFER CIRCULAR ---
  // Hacemos una lectura inicial para rellenar el buffer con un valor real
  // y evitar que la media empiece arrastrando ceros.
  uint16_t lecturaInicial = sensor.readRangeSingleMillimeters();
  if (lecturaInicial > 2000 || sensor.timeoutOccurred()) lecturaInicial = 2000;
  
  for (int i = 0; i < TAMANO_BUFFER; i++) {
    buffer[i] = (float)lecturaInicial;
  }
  sumaAcumulada = (float)lecturaInicial * TAMANO_BUFFER;
  distanciaFiltrada = (float)lecturaInicial;
  bufferLleno = true; // Ya está pre-llenado con datos válidos

  // Cabecera para el Serial Plotter
  Serial.println("Consigna_mm,Distancia_Filtrada_mm,Duty_Cycle_Porcentaje");
}

void loop() {
  // 1. Leer comandos del Monitor Serie para actualizar la consigna
  leerConsignaSerial();

  // 2. Subprograma de Control PID (Muestreo periódico constante)
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior >= intervaloControl) {
    tiempoAnterior = tiempoActual;
    
    leerSensor();        // Aquí se lee, se añade al buffer y se calcula la media móvil
    ejecutarControlPID(); // El PID ahora usa la señal limpia
    enviarDatosPlotter();
  }
}

// ==========================================
// SUBPROGRAMA 1: Lectura y Filtrado del Sensor Láser (Buffer Circular)
// ==========================================
void leerSensor() {
  uint16_t lecturaRaw = sensor.readRangeSingleMillimeters();
  float lecturaValida = 0.0;

  // Validación de errores de lectura
  if (sensor.timeoutOccurred() || lecturaRaw > 2000) {
    lecturaValida = 2000.0; 
  } else {
    lecturaValida = (float)lecturaRaw;
  }

  // --- FILTRO DE MEDIA MÓVIL MEDIANTE BUFFER CIRCULAR ---
  
  // 1. Restamos de la suma el valor viejo que va a salir del buffer
  sumaAcumulada -= buffer[indiceBuffer];
  
  // 2. Metemos la nueva lectura válida en la posición actual del buffer
  buffer[indiceBuffer] = lecturaValida;
  
  // 3. Sumamos la nueva lectura a nuestra suma acumulada
  sumaAcumulada += lecturaValida;
  
  // 4. Movemos el índice de forma cíclica usando el operador módulo
  indiceBuffer = (indiceBuffer + 1) % TAMANO_BUFFER;
  
  // 5. Calculamos la media pura del buffer (distancia con ruido mitigado)
  float distanciaMediaPura = sumaAcumulada / TAMANO_BUFFER;

  // 6. Calculamos el error sistemático que tiene el sensor a esta distancia exacta
  float errorSensor = calcularErrorInterpolado(distanciaMediaPura);

  // 7. Compensamos la medida restando el error medido. 
  // Esta es la distancia corregida y ultra-precisa que usará el PID.
  distanciaFiltrada = distanciaMediaPura - errorSensor;
}

// ==========================================
// SUBPROGRAMA 2: Lazo Cerrado PID y Escritura PWM
// ==========================================
void ejecutarControlPID() {
  // 1. Calcular el error actual USANDO LA DISTANCIA FILTRADA
  float error = consigna - distanciaFiltrada;
  
  // 2. Acción Proporcional
  float P = -Kp * error; 
  
  // 3. Acción Integral con Anti-Windup condicional
  if (porcentajePWM > 0.0 && porcentajePWM < 100.0) {
    integral += ((error + errorAnterior) / 2) * dt;
  }
  float I = -Ki * integral;
  
  // 4. Acción Derivativa 
  float derivat = (error - errorAnterior) / dt;
  float D = -Kd * derivat;
  
  // 5. Ecuación global PID con punto de operación nominal al 78%
  porcentajePWM = BIAS + P + I + D;
  
  // --- SATURACIÓN ESTRICTA (0% - 100%) ---
  if (porcentajePWM > 100.0) porcentajePWM = 100.0;
  if (porcentajePWM < 0.0)   porcentajePWM = 0.0;
  
  // Guardar el error actual para el próximo ciclo
  errorAnterior = error;
  
  // Convertir el porcentaje final al rango físico del Timer 1 (0 a 320)
  int valorOCR1A = (porcentajePWM / 100.0) * 320;
  
  // Aplicar acción de control al registro del microcontrolador
  OCR1A = valorOCR1A;
}

// ==========================================
// FUNCIÓN: Captura de nueva consigna por Serial
// ==========================================
void leerConsignaSerial() {
  if (Serial.available() > 0) {
    float nuevaConsigna = Serial.parseFloat();
    if (nuevaConsigna > 0 && nuevaConsigna <= 2000) {
      consigna = nuevaConsigna;
    }
  }
}

// ==========================================
// FUNCIÓN AUXILIAR: Graficación en Serial Plotter
// ==========================================
void enviarDatosPlotter() {
  Serial.print(consigna);
  Serial.print(",");
  Serial.print(distanciaFiltrada); // Graficamos la señal limpia
  Serial.print(",");
  Serial.println(porcentajePWM);
}

// ==========================================
// FUNCIÓN: Interpolación Lineal para Cálculo del Error
// ==========================================
float calcularErrorInterpolado(float distancia) {
  // Caso 1: Por debajo del primer punto calibrado
  if (distancia <= puntosCalibracion[0]) {
    return erroresMedidos[0];
  }
  
  // Caso 2: Por encima del último punto calibrado
  if (distancia >= puntosCalibracion[NUM_PUNTOS - 1]) {
    return erroresMedidos[NUM_PUNTOS - 1];
  }
  
  // Caso 3: Buscar el intervalo correspondiente (Interpolación lineal)
  for (int i = 0; i < NUM_PUNTOS - 1; i++) {
    if (distancia >= puntosCalibracion[i] && distancia <= puntosCalibracion[i+1]) {
      // Puntos del tramo actual
      float x0 = puntosCalibracion[i];
      float x1 = puntosCalibracion[i+1];
      float y0 = erroresMedidos[i];
      float y1 = erroresMedidos[i+1];
      
      // Ecuación de la recta: y = y0 + ((x - x0) * (y1 - y0) / (x1 - x0))
      float errorInterpolado = y0 + ((distancia - x0) * (y1 - y0) / (x1 - x0));
      return errorInterpolado;
    }
  }
  return 0.0; // Salvaguarda
}