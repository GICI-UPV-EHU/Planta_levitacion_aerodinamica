#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  
  // Forzamos un pequeño retraso para que el sensor se estabilice eléctricamente
  delay(500); 
  
  Wire.begin();

  // Inicializamos el sensor
  if (!sensor.init()) {
    Serial.println("Error al iniciar el VL53L0X");
    while (1);
  }

  // Aumentamos el tiempo de espera (timeout) a 1000 milisegundos
  sensor.setTimeout(1000);
  
  // Configuramos el sensor en modo de largo alcance y alta precisión (opcional pero recomendado)
  sensor.setSignalRateLimit(0.1);
  sensor.setMeasurementTimingBudget(200000); // 200ms por lectura para asegurar estabilidad

  Serial.println("¡Sensor VL53L0X configurado con éxito!");
}

void loop() {
  // Leemos la distancia en modo "Single" (un disparo a la vez)
  uint16_t distancia = sensor.readRangeSingleMillimeters();

  // Verificamos si hubo un timeout real en esta lectura
  if (sensor.timeoutOccurred()) {
    Serial.println(" -> Lectura perdida (Timeout) <- ");
  } else {
    Serial.print("Distancia: ");
    if (distancia > 2000) {
      Serial.println("Fuera de rango (> 2m)");
    } else {
      Serial.print(distancia);
      Serial.println(" mm");
    }
  }

  delay(300); // Pausa de 300ms entre lecturas para no saturar el bus
}