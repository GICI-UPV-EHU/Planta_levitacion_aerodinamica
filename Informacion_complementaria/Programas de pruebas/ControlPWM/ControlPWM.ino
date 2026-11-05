// El Pin 9 está ligado al canal A del Timer 1 (OC1A)
const int pinPWM = 9;

// Variable para el Duty Cycle (rango de 0 a 320 para 25kHz)
int valorPWM = 0; 

// El paso de incremento/decremento (aprox. 10% de 320 es 32)
const int paso = 1; // Modificado a 32 para que realmente aumente/disminuya un ~10%

void setup() {
  Serial.begin(9600);
  
  // Configurar el Pin 9 como salida
  pinMode(pinPWM, OUTPUT);
  
  // --- CONFIGURACIÓN DEL TIMER 1 PARA 25 kHz ---
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Configurar Modo 10: PWM con corrección de fase y frecuencia (TOP en ICR1)
  TCCR1A |= (1 << COM1A1);              // Limpiar OC1A al subir, setear al bajar
  TCCR1B |= (1 << WGM13) | (1 << CS10); // Prescaler = 1 (sin división de reloj)
  
  // Definir el valor TOP en el registro ICR1 para fijar la frecuencia a 25 kHz
  // TOP = 16,000,000 Hz / (2 * 1 * 25,000 Hz) = 320
  ICR1 = 320; 

  // Inicializar el ventilador apagado (0% Duty Cycle)
  OCR1A = 0; 
  
  Serial.println("--- Control de Ventilador Delta (25 kHz) ---");
  Serial.println("Envia un numero de 0 a 100 para fijar el porcentaje directamente.");
  Serial.println("O envia '+' / '-' para ajustar la velocidad gradualmente.");
  mostrarEstado();
}

void loop() {
  if (Serial.available() > 0) {
    // Miramos el primer carácter sin sacarlo del búfer para saber si es un comando o un número
    char primerCaracter = Serial.peek(); 
    
    if (primerCaracter == '+' || primerCaracter == '-') {
      // Si es un signo, lo leemos de forma normal
      char comando = Serial.read();
      
      if (comando == '+') {
        valorPWM += paso;
        if (valorPWM > 320) valorPWM = 0; // Límite superior 100%
      } 
      else if (comando == '-') {
        valorPWM -= paso;
        if (valorPWM < 0) valorPWM = 320;     // Límite inferior 0%
      }
      
      // Aplicar el nuevo Duty Cycle directamente al registro del Timer 1
      OCR1A = valorPWM;
      mostrarEstado();
    } 
    else if (isDigit(primerCaracter)) {
      // Si el primer carácter es un número, leemos el entero completo (ej. "44")
      int porcentajeEntrada = Serial.parseInt();
      
      // Aseguramos que el porcentaje esté entre 0 y 100%
      porcentajeEntrada = constrain(porcentajeEntrada, 0, 100);
      
      // Convertimos el porcentaje (0-100) al rango del registro del Timer (0-320)
      valorPWM = map(porcentajeEntrada, 0, 100, 0, 320);
      
      // Aplicar al registro del Timer 1
      OCR1A = valorPWM;
      mostrarEstado();
    } 
    else {
      // Descartar cualquier otro carácter basura (como saltos de línea \n o \r)
      Serial.read();
    }
  }
}

void mostrarEstado() {
  // Calculamos el porcentaje real en base al nuevo rango (0 a 320)
  int porcentaje = map(valorPWM, 0, 320, 0, 100);
  
  Serial.print("Velocidad Ventilador: ");
  Serial.print(porcentaje);
  Serial.print("% (Registro OCR1A: ");
  Serial.print(valorPWM);
  Serial.println(")");
}