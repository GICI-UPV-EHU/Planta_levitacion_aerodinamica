#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

// El Pin 9 está ligado al canal A del Timer 1 (OC1A)
const int pinPWM = 9;

// Variable para la altura del objeto
float Altura = 0.0;

// --- CONFIGURACIÓN DEL CONTROLADOR PID (CANALES DESDE EXCEL) ---
float consigna    = 0.0;      // CH1: Consigna en milímetros
float Kp          = 0.05;       // CH2: Ganancia Proporcional
float Ki          = 0.0;       // CH3: Ganancia Integral
float Kd          = 0.0;       // CH4: Ganancia Derivativa
float BIAS        = 70.00;      // Punto de operación nominal
float Duty_Cicle  = 70.00;      // Duty Cicle de la señal PWM

// Variables internas del PID
float errorAnterior = 0.0;
float integral = 0.0;

// Temporización estricta para el cálculo del PID (dt constante)
unsigned long tiempoAnterior = 0;
const float dt = 0.1;               // Período de muestreo en segundos (100 ms)
const long intervaloControl = 100;  // Período de muestreo en milisegundos (100 ms)


// --- CONFIGURACIÓN DE LA CALIBRACIÓN (INTERPOLACIÓN) ---
const int NUM_PUNTOS = 7;
// Puntos de control medidos (deben estar estrictamente ordenados de menor a mayor)
const float puntosCalibracion[NUM_PUNTOS] = {0.0, 100.0, 200.0, 300.0, 400.0, 500.0, 600.0};
// Errores medidos en cada uno de esos puntos
const float erroresMedidos[NUM_PUNTOS]    = {0.0, -40.0, -60.0, -80.0, -85.0, -80.0, -40.0};

void setup() {
  Serial.begin(9600);     //Inicia la comunicación serie a una velocidad de 9600 baudios (bits por segundo)
  Serial.setTimeout(50);  // Establece el tiempo máximo de espera en 50 milisegundos para funciones de lectura serie como readStringUntil().
  
  delay(500);             //Espera 500ms
  Wire.begin();           //Inicializa la librería Wire, habilitando el puerto I2C del Arduino.

  if (!sensor.init()) {
    Serial.println ("Error_Inicializacion_VL53L0X");
    while (1);        //Bloqueo del programa
  }

  sensor.setTimeout(1000);                  //Configura un tiempo límite de 1000 ms para que el sensor devuelva una medida. Si sobrepasa este tiempo, la librería notificará que ocurrió un timeout.
  sensor.setSignalRateLimit(0.1);           //Ajusta el límite mínimo de tasa de retorno de señal a 0.1 MCPS (Mega Counts Per Second).
  sensor.setMeasurementTimingBudget(50000); //Establece el tiempo dedicado a cada medición en 50.000 microsegundos (50 ms). A mayor tiempo de medición, más precisa es la lectura del láser.

  pinMode(pinPWM, OUTPUT);                  //Configura el Pin 9 del Arduino como una salida digital para enviar la señal PWM al actuador.
  
  //Configuración del Timer1 para 25kHz
  TCCR1A = 0;                               //Limpia los bits del registro de control A del Timer 1, eliminando cualquier configuración previa por defecto de Arduino.
  TCCR1B = 0;                               //Limpia los bits del registro de control B del Timer 1.
  TCNT1  = 0;                               //Reinicia el contador interno del Timer 1 a 0.
  TCCR1A |= (1 << COM1A1);                  //Activa el modo PWM no invertido en el canal A (OC1A / Pin 9). La salida estará en HIGH cuando el contador empiece y pasará a LOW cuando alcance el valor de comparación (OCR1A).
  TCCR1B |= (1 << WGM13) | (1 << CS10);     //Configura el modo PWM rápido con frecuencia definida por el registro ICR1 (WGM13) y establece un divisor de reloj (prescaler) de 1 (CS10), utilizando la frecuencia base de 16 MHz.
  ICR1 = 320;                               //Establece el valor máximo del contador en 320. Esto fija la frecuencia del PWM exactamente en 25 kHz (16MHz / 320 = 25.000Hz) y define la resolución de la señal de 0 a 320 pasos.

  // Cabecera para lectura en Excel Data Streamer o Serial Plotter
  // Serial.println("Consigna (mm), Altura, Duty Cycle");
}

void loop() {

  // 1. Leer trama multinivel enviada desde Excel (CH1, CH2, CH3, CH4)
  leerExcel();

  // 2. Subprograma de Control PID (Muestreo periódico constante)
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= intervaloControl) {
    tiempoAnterior = tiempoActual;
    
    leerSensor();         // Muestreo y filtrado con compensación

    ejecutarControlPID(); // Cálculo de la ley de control

    enviarDatosPlotter(); // Envío de salidas a Excel / Serial

  }
}


// SUBPROGRAMA 1: Lectura y Filtrado del Sensor Láser

void leerSensor() {
  uint16_t lectura = sensor.readRangeSingleMillimeters();    //Solicita una medida puntual al sensor láser VL53L0X. El resultado se guarda como un entero sin signo de 16 bits (uint16_t), representando la distancia en milímetros desde el sensor hasta el objeto.
  float lecturaValida = 0.0;

  // Validación de errores de lectura
  if (sensor.timeoutOccurred() || lectura > 700) {
    lecturaValida = 700.0; 
  } else {
    lecturaValida = (float)lectura;
  }

  // Calcular la altura
  Altura = 655.0 - lecturaValida;

  // Calculamos el error sistemático por interpolación
  float errorSensor = calcularErrorInterpolado(Altura);

  // Corregir a la altura el error del sensor
  Altura = Altura - errorSensor;
}



// SUBPROGRAMA 2: Lazo Cerrado PID y Escritura PWM

void ejecutarControlPID() {
  // 1. Calcular el error actual usando la Altura
  float error = consigna - Altura;
  
  // 2. Acción Proporcional
  float P = Kp * error; 
  
  // 3. Acción Integral con Anti-Windup condicional
  if (Duty_Cicle > 0.0 && Duty_Cicle < 100.0) {
    integral += ((error + errorAnterior) / 2.0) * dt;
  }

  float I = Ki * integral;
  
  // 4. Acción Derivativa 
  float derivat = (error - errorAnterior) / dt;
  float D = Kd * derivat;
  
  // 5. Ecuación global PID con punto de operación nominal
  Duty_Cicle = BIAS + P + I + D;
  
  // --- SATURACIÓN ESTRICTA (0% - 100%) ---
  if (Duty_Cicle > 100.0) Duty_Cicle = 100.0;
  if (Duty_Cicle < 0.0)   Duty_Cicle = 0.0;
  
  // Guardar el error actual para el próximo ciclo
  errorAnterior = error;
  
  // Convertir el porcentaje final al rango físico del Timer 1 (0 a 320) y Aplicar acción de control al registro del microcontrolador
  OCR1A = (Duty_Cicle / 100.0) * 320;
  
}


// SUBPROGRAMA 3: Captura de Trama desde Excel (CH1, CH2, CH3, CH4)

void leerExcel() {
  if (Serial.available() > 0) {                               //Comprueba si hay datos pendientes de leer en el búfer del puerto serie (si la cantidad de bytes disponibles es mayor que cero).
    String cadenaRecibida = Serial.readStringUntil('\n');     //Lee todos los caracteres recibidos en el puerto serie hasta encontrar un salto de línea ('\n'). Guarda todo esa cadena de texto en la variable cadenaRecibida.
    cadenaRecibida.trim();                                    // Limpiar espacios/retornos de carro (\r)

    if (cadenaRecibida.length() > 0) {                        //Verifica que la cadena no esté vacía después de haber sido limpiada.
      char buffer[64];                                        //Crea un arreglo de caracteres (un array de tipo char) de 64 elementos.
      cadenaRecibida.toCharArray(buffer, sizeof(buffer));     //Copia el contenido del objeto cadenaRecibida dentro del arreglo buffer, asegurando no sobrepasar el límite de 64 bytes.

      // Extraer CH1: Consigna
      char* token = strtok(buffer, ",");                      //Utiliza la función strtok() para cortar el buffer en la primera coma (,). Retorna un puntero al primer subtexto (que corresponde al parámetro de la consigna).
      if (token != NULL) {
        float val = atof(token);                              //Convierte el texto extraído (token) a un número de punto flotante (float) mediante la función atof() (ascii to float).
        if (val >= 0 && val <= 660) consigna = val;
      }

      // Extraer CH2: Kp
      token = strtok(NULL, ",");                              //Llama nuevamente a strtok() pasando NULL como primer argumento para continuar extrayendo el siguiente elemento del mismo buffer (después de la segunda coma).
      if (token != NULL) Kp = atof(token);

      // Extraer CH3: Ki
      token = strtok(NULL, ",");
      if (token != NULL) Ki = atof(token);

      // Extraer CH4: Kd
      token = strtok(NULL, ",");
      if (token != NULL) Kd = atof(token);
    }
  }
}


// SUBPROGRAMA 4: Graficación / Salida de Datos

void enviarDatosPlotter() {
  Serial.print(consigna);
  Serial.print(",");
  Serial.print(Altura);
  Serial.print(",");
  Serial.print(Duty_Cicle);
  Serial.print(",");
  Serial.print(Kp);
  Serial.print(",");
  Serial.print(Ki);
  Serial.print(",");
  Serial.println(Kd);
}


// SUBPROGRAMA 5: Interpolación Lineal para Cálculo del Error

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