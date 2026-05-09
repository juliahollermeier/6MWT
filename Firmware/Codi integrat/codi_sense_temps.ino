#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>

#define SERVICE_UUID         "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID_TX "87654321-4321-4321-4321-210987654321"
#define CHARACTERISTIC_UUID_RX "11223344-5566-7788-9900-aabbccddeeff"

// Configuración BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Variables BLE
BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristicTX = nullptr;
BLECharacteristic* pCharacteristicRX = nullptr;
bool deviceConnected = false;

// Variables de estado y medición
bool isRunning = false;
unsigned long startTime = 0;
int turnCount = 0; // Medias vueltas de 180°
int stepCount = 0; // Pasos desde el último giro

// Variables para detección de giros
bool firstReading = true;
int tracking_axis = 0; 
float lastHeading = 0;
float accumulatedRotation = 0;
const float TURN_THRESHOLD = 120.0;  // Umbral para medias vueltas
const float NOISE_THRESHOLD = 2.5;   // Filtro para ignorar el tambaleo de la cadera
unsigned long lastTurnTime = 0;      // Temporizador para no contar giros dobles

// Variables para podómetro (Valores confirmados)
unsigned long lastStepTime = 0;
const float STEP_THRESHOLD = 3.0;    // Sensibilidad calibrada
const int STEP_TIMEOUT = 450;        // Tiempo de bloqueo calibrado

// Normaliza la diferencia angular al rango [-180, 180]
float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

// --- Callback para recibir mensajes del móvil ---
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String valor = pCharacteristic->getValue().c_str();
    valor.trim();
    valor.toLowerCase();
    
    if (valor == "go") {
      isRunning = true;
      startTime = millis();
      turnCount = 0;
      stepCount = 0;
      accumulatedRotation = 0;
      firstReading = true; // Forzamos recalibrar el eje horizontal al arrancar
      
      Serial.println("Medición iniciada");
      pCharacteristicTX->setValue("Go!");
      pCharacteristicTX->notify();
    }
    else if (valor == "stop") {
      if (isRunning) {
        isRunning = false;
        float timeSeconds = (millis() - startTime) / 1000.0;
        
        // Formato comprimido: vueltas; pasos
        char resultStr[20];
        snprintf(resultStr, sizeof(resultStr), "%d;%d", turnCount, stepCount);
        
        Serial.println("Medición finalizada");
        Serial.println(resultStr);
        
        pCharacteristicTX->setValue(resultStr);
        pCharacteristicTX->notify();
      }
    }
  }
};

// --- Callback para conexión/desconexión ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Cliente conectado");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    isRunning = false;
    Serial.println("Cliente desconectado");
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!bno.begin()) {
    Serial.println("Error: No se detecta el BNO055.");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 inicializado correctamente");

  BLEDevice::init("ESP32_Grup4");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristicTX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristicTX->addDescriptor(new BLE2902());

  pCharacteristicRX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristicRX->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Esperando conexión BLE. Envía 'go' o 'stop'.");
}

void loop() {
  static unsigned long lastSampleTime = 0;
  
  if (isRunning && deviceConnected) {
    
    // Bucle a 50Hz (cada 20ms)
    if (millis() - lastSampleTime >= 20) {
      lastSampleTime = millis();
      
      // ==========================================
      // 1. PODÓMETRO
      // ==========================================
      imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      float mag = sqrt((lin_accel.x() * lin_accel.x()) + 
                       (lin_accel.y() * lin_accel.y()) + 
                       (lin_accel.z() * lin_accel.z()));
      
      // Filtro de fuerza y tiempo para contar el paso
      if (mag > STEP_THRESHOLD && (millis() - lastStepTime > STEP_TIMEOUT)) {
        stepCount++;
        lastStepTime = millis();
        Serial.print("Paso #");
        Serial.println(stepCount);
      }

      // ==========================================
      // 2. GIROS (Con Anti-Gimbal Lock)
      // ==========================================
      imu::Quaternion q = bno.getQuat();

      // Extracción de ejes proyectados sobre la Tierra
      float x_ex = 1.0 - 2.0 * (q.y()*q.y() + q.z()*q.z());
      float x_ey = 2.0 * (q.x()*q.y() + q.w()*q.z());
      float x_ez = 2.0 * (q.x()*q.z() - q.w()*q.y());

      float y_ex = 2.0 * (q.x()*q.y() - q.w()*q.z());
      float y_ey = 1.0 - 2.0 * (q.x()*q.x() + q.z()*q.z());
      float y_ez = 2.0 * (q.y()*q.z() + q.w()*q.x());

      float z_ex = 2.0 * (q.x()*q.z() + q.w()*q.y());
      float z_ey = 2.0 * (q.y()*q.z() - q.w()*q.x());
      float z_ez = 1.0 - 2.0 * (q.x()*q.x() + q.y()*q.y());

      if (firstReading) {
        // Seleccionar el eje que esté más horizontal respecto al suelo
        if (fabs(x_ez) < fabs(y_ez) && fabs(x_ez) < fabs(z_ez)) tracking_axis = 0;
        else if (fabs(y_ez) < fabs(x_ez) && fabs(y_ez) < fabs(z_ez)) tracking_axis = 1;
        else tracking_axis = 2;
        
        if (tracking_axis == 0) lastHeading = atan2(x_ey, x_ex) * 180.0 / PI;
        if (tracking_axis == 1) lastHeading = atan2(y_ey, y_ex) * 180.0 / PI;
        if (tracking_axis == 2) lastHeading = atan2(z_ey, z_ex) * 180.0 / PI;
        
        firstReading = false;
      } else {
        float currentHeading = 0;
        if (tracking_axis == 0) currentHeading = atan2(x_ey, x_ex) * 180.0 / PI;
        if (tracking_axis == 1) currentHeading = atan2(y_ey, y_ex) * 180.0 / PI;
        if (tracking_axis == 2) currentHeading = atan2(z_ey, z_ex) * 180.0 / PI;

        float deltaAngle = normalizeAngle(currentHeading - lastHeading);

        if (fabs(deltaAngle) > NOISE_THRESHOLD) {
          accumulatedRotation += deltaAngle;
          lastHeading = currentHeading;

          // Detectar media vuelta hacia la derecha o izquierda
          if (abs(accumulatedRotation) >= TURN_THRESHOLD) {
            if (millis() - lastTurnTime > 1500) {  // Cooldown de 1.5 segundos
              turnCount++;
              lastTurnTime = millis();
              
              // --- NUEVA LÍNEA: Resetear contador de pasos al girar ---
              stepCount = 0;
              
              Serial.println("¡Media vuelta detectada! (Pasos reseteados a 0)");
            }
            accumulatedRotation = 0; // Limpiar acumulador tras el giro
          }
        }
      }
    }
  }
}