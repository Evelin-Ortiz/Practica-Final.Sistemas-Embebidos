/*
 * Nombre del archivo: Practica Final Sistemas Embebidos Tarjeta 2
 * Descripción:
 * Este programa implementa un sistema de adquisición de datos utilizando un Heltec LoRa V3.
 * Recoge información de un sensor DHT11, un GPS, un potenciómetro y un sensor PIR, 
 * mostrando los datos en una pantalla OLED y transmitiéndolos mediante LoRaWAN.
 * También incluye control de LEDs y un contador de bloqueos detectados por el PIR.
 * 
 * Autor: Evelin Ortiz - Oscar Matabajoy
 * Fecha: 20/11/24
 */

#include "LoRaWan_APP.h"
#include <DHT.h>
#include "HT_SSD1306Wire.h"
#include <TinyGPS++.h>

// Configuración de pines y dispositivos
#define DHTPIN 38         // Pin donde está conectado el DHT11
#define DHTTYPE DHT11     // Tipo de sensor
#define LED_ROJO 45       // Pin para el LED rojo
#define LED_AZUL 42       // Pin para el LED azul
#define LED_BLANCO 41     // Pin para el LED blanco del TIMER
#define POT_PIN 7         // Pin para el potenciometro
#define PIR_PIN 34        // Pin para el sensor FC-51
#define BUTTON_PIN   36   // Pin para el botón reset
// Configuración del GPS
#define RXD2 6
#define TXD2 5
#define GPS_BAUD 9600
// Inicialización de instancias
DHT dht(DHTPIN, DHTTYPE);
// Instancia para Serial2 (GPS)
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
// Configuración de la pantalla OLED
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Variables de temporizador y LoRaWAN
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xBE, 0x93 };
uint8_t appEui[] = { 0xEC, 0xFF, 0x9F, 0xFA, 0x12, 0xF4, 0x00, 0x00 };
uint8_t appKey[] = { 0xB8, 0x06, 0x63, 0xA3, 0xF7, 0xE9, 0xA8, 0x2C, 0xD9, 0x14, 0x6A, 0x7C, 0xC0, 0x92, 0x29, 0xEA };
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;
uint16_t userChannelsMask[6]={ 0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000 };
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_C;
uint32_t appTxDutyCycle = 5000;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;
RTC_DATA_ATTR bool firstrun = true;
// Variables globales
float temperatura = 0; 
float humedad =0;
int potValue = 0; 
double latitud = 0;
double longitud = 0;
int grupo=7;
int num_tar=2;
volatile unsigned long contador = 0;  // Contador para ISR
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;
// Variables globales para manejar la detección del PIR
volatile bool pirActivo = false;  // Estado actual del PIR
volatile bool mostrarBloqueo = false;  // Bandera para mostrar el mensaje de bloqueo
hw_timer_t *timerBloqueo = NULL;  // Temporizador para el manejo del mensaje de bloqueo
RTC_DATA_ATTR volatile int cantidadBloqueos= 0;  // Contador de bloqueos


// ISR para el control del LED blanco
void IRAM_ATTR onTimer() {
  contador++;  // Incrementar contador en cada milisegundo
  if (digitalRead(LED_BLANCO) && contador >= 1) {  // Encendido por 250 ms
    contador = 0;
    digitalWrite(LED_BLANCO, LOW);
  } else if (!digitalRead(LED_BLANCO) && contador >= 3) {  // Apagado por 750 ms
    contador = 0;
    digitalWrite(LED_BLANCO, HIGH);
  }
}
/*
 * Función: mostrarPantallaTemperatura
 * Descripción: Muestra los datos de temperatura, humedad y potenciómetro en la OLED.
 * Parámetros:
 *     - grupo: Grupo del sistema.
 *     - num_tar: Número de la tarjeta.
 *     - temperatura: Valor de la temperatura actual.
 *     - humedad: Valor de la humedad actual.
 *     - potValue: Valor del potenciómetro.
 */
void mostrarPantallaTemperatura(int grupo, int num_tar, float temperatura, float humedad, int potValue) {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 0, "Grupo: " + String(grupo) + " Tarjeta: " + String(num_tar));
  oledDisplay.drawString(0, 16, "Temp: " + String(temperatura, 2)); 
  oledDisplay.drawString(0, 32, "Hum: " + String(humedad, 2));   
  oledDisplay.drawString(0, 48, "Voltaje: " + String(potValue));
  oledDisplay.display();
}
/*
 * Función: mostrarPantallaGPS
 * Descripción: Muestra los datos de GPS y bloqueos en la OLED.
 * Parámetros:
 *     - latitud: Latitud actual del GPS.
 *     - longitud: Longitud actual del GPS.
 *     - cantidadBloqueos: Número de bloqueos detectados.
 */
void mostrarPantallaGPS(double latitud, double longitud, int cantidadBloqueos) {
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 0, "Latitud: " + String(latitud, 6));
  oledDisplay.drawString(0, 16, "Longitud: " + String(longitud, 6));
  oledDisplay.drawString(0, 32, "Bloqueos: " + String(cantidadBloqueos));
  oledDisplay.display();
}

/*
 * Función: PantallaCompleta
 * Descripción: Muestra secuencialmente los datos de temperatura, humedad, potenciómetro, GPS y bloqueos.
 * Parámetros: Mismos que mostrarPantallaTemperatura y mostrarPantallaGPS.
 */
void PantallaCompleta(int grupo, int num_tar, float temperatura, float humedad, int potValue,double latitud, double longitud, int cantidadBloqueos) {
  mostrarPantallaTemperatura(grupo, num_tar, temperatura, humedad, potValue);
  delay(2000);
  mostrarPantallaGPS(latitud,longitud,cantidadBloqueos);
}

// Interrupción para el temporizador de bloqueo
void IRAM_ATTR onBloqueoTimer() {
    mostrarBloqueo = false;  // Finalizar la visualización del mensaje
    timerAlarmDisable(timerBloqueo);  // Detener el temporizador
}
// Configurar temporizador para manejar la visualización de bloqueos
void configurarTemporizadorBloqueo() {
    timerBloqueo = timerBegin(1, 80, true);  // Inicializar temporizador con prescaler 80
    timerAttachInterrupt(timerBloqueo, &onBloqueoTimer, true);  // Asociar interrupción
    timerAlarmWrite(timerBloqueo, 250000, false);  // Configurar para 250 ms
}

// Detectar movimiento con el sensor PIR
void detectarBloqueo() {
    bool estadoPIR = digitalRead(PIR_PIN) == LOW;  // Leer el estado actual del PIR
    if (estadoPIR && !pirActivo && (millis() - lastDebounceTime) > debounceDelay) {
        // Solo incrementar si el PIR está activo y antes estaba inactivo
        pirActivo = true;  // Actualizar estado del PIR
        cantidadBloqueos++;  // Incrementar la variable cada vez que se detecte movimiento
        lastDebounceTime = millis();  // Actualiza el tiempo del debounce

        // Configurar el temporizador para 250 ms
        mostrarBloqueo = true;  // Activar la visualización del mensaje
        timerAlarmEnable(timerBloqueo);  // Habilitar el temporizador

        // Mostrar el mensaje "Bloqueo x" en la pantalla OLED
        oledDisplay.clear();
        oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
        oledDisplay.drawString(0, 0, "Bloqueo " + String(cantidadBloqueos));
        oledDisplay.display();
    } else if (!estadoPIR) {
        pirActivo = false;  // Resetear el estado del PIR si no está activo
    }
}

/*
 * Función: prepareTxFrame
 * Descripción: Prepara los datos a transmitir por LoRaWAN en un formato específico.
 * Parámetros:
 *     - port: Puerto de comunicación.
 */
static void prepareTxFrame(uint8_t port)
{
    temperatura = dht.readTemperature();
    humedad = dht.readHumidity();
    potValue = analogRead(POT_PIN);  // Leer el valor del potenciómetro (0 - 4095)
    latitud = gps.location.lat();
    longitud = gps.location.lng();
    // Convertir valores a formato de transmisión
    long temp_s = temperatura * 10000000;
    long hum_s = humedad * 10000000;
    long lati_s = latitud * 10000000;
    long longi_s= longitud * 10000000;
    // Control de LEDs
    digitalWrite(LED_ROJO, temperatura > 25 ? HIGH : LOW);
    digitalWrite(LED_AZUL, humedad < 40 ? HIGH : LOW);
    // Procesar los datos del GPS
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    // Lógica para empaquetar datos...
      appDataSize = 32;//AppDataSize max value is 64
      appData[0] = (byte) ((lati_s & 0xFF000000) >> 24 );
      appData[1] = (byte) ((lati_s & 0x00FF0000) >> 16 );
      appData[2] = (byte) ((lati_s & 0x0000FF00) >> 8 );
      appData[3] = (byte) ((lati_s & 0X000000FF));
      appData[4] = (byte) ((longi_s & 0xFF000000) >> 24 );
      appData[5] = (byte) ((longi_s & 0x00FF0000) >> 16 );
      appData[6] = (byte) ((longi_s & 0x0000FF00) >> 8 );
      appData[7] = (byte) ((longi_s & 0X000000FF));
      appData[8] = (byte) ((potValue & 0xFF000000) >> 24 );
      appData[9] = (byte) ((potValue & 0x00FF0000) >> 16 );
      appData[10] = (byte) ((potValue & 0x0000FF00) >> 8 );
      appData[11] = (byte) ((potValue & 0X000000FF));
      appData[12] = (byte) ((hum_s & 0xFF000000) >> 24 );
      appData[13] = (byte) ((hum_s & 0x00FF0000) >> 16 );
      appData[14] = (byte) ((hum_s & 0x0000FF00) >> 8 );
      appData[15] = (byte) ((hum_s & 0X000000FF));
      appData[16] = (byte) ((temp_s & 0xFF000000) >> 24 );
      appData[17] = (byte) ((temp_s & 0x00FF0000) >> 16 );
      appData[18] = (byte) ((temp_s & 0x0000FF00) >> 8 );
      appData[19] = (byte) ((temp_s & 0X000000FF));
      appData[20] = (byte) ((grupo & 0xFF000000) >> 24 );
      appData[21] = (byte) ((grupo & 0x00FF0000) >> 16 );
      appData[22] = (byte) ((grupo & 0x0000FF00) >> 8 );
      appData[23] = (byte) ((grupo & 0X000000FF));
      appData[24] = (byte) ((num_tar & 0xFF000000) >> 24 );
      appData[25] = (byte) ((num_tar & 0x00FF0000) >> 16 );
      appData[26] = (byte) ((num_tar & 0x0000FF00) >> 8 );
      appData[27] = (byte) ((num_tar & 0X000000FF));
      appData[28] = (byte) ((cantidadBloqueos & 0xFF000000) >> 24 );
      appData[29] = (byte) ((cantidadBloqueos & 0x00FF0000) >> 16 );
      appData[30] = (byte) ((cantidadBloqueos & 0x0000FF00) >> 8 );
      appData[31] = (byte) ((cantidadBloqueos & 0X000000FF));
}

void setup() {
    Serial.begin(115200);
    hw_timer_t *myTimer = NULL; 
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);  // Inicializa el puerto serie para el GPS
    dht.begin();
    pinMode(LED_ROJO, OUTPUT);
    pinMode(LED_AZUL, OUTPUT);
    pinMode(LED_BLANCO, OUTPUT);
    pinMode(PIR_PIN, INPUT);  // Configurar el pin del PIR como entrada
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configurar el botón como entrada con resistencia pull-up
    Mcu.begin();
    if (firstrun) {
        LoRaWAN.displayMcuInit();
        firstrun = false;
    }
    deviceState = DEVICE_STATE_INIT;
    // Configuración del temporizador
    myTimer = timerBegin(0, 80, true);               // Inicializa el temporizador
    timerAttachInterrupt(myTimer, &onTimer, true);   // Asocia el temporizador a la interrupción
    timerAlarmWrite(myTimer, 250000, true);          // Configura la alarma cada 250 ms
    timerAlarmEnable(myTimer);  
    // Configuración del temporizador de bloqueos
    configurarTemporizadorBloqueo();
    // Inicializa la pantalla OLED
    oledDisplay.init();
    oledDisplay.setFont(ArialMT_Plain_10);
}

void loop(){

   // Lógica para detectar bloqueos
    detectarBloqueo();

    // Detectar un toque en el botón de reset
    if (digitalRead(BUTTON_PIN) == LOW) {
        cantidadBloqueos = 0;  // Reiniciar la cantidad de bloqueos
        delay(250);  // Pequeña pausa para evitar rebotes
    }
    switch (deviceState)
    {
        case DEVICE_STATE_INIT:
        {
#if(LORAWAN_DEVEUI_AUTO)
            LoRaWAN.generateDeveuiByChipID();
#endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            LoRaWAN.displayJoining();
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND:
        {
            LoRaWAN.displaySending();
            prepareTxFrame(appPort);
            // Si no hay bloqueo, continuar mostrando datos normalmente
            if (!mostrarBloqueo) {
              PantallaCompleta(grupo, num_tar, temperatura, humedad, potValue, latitud, longitud, cantidadBloqueos);
            }
            LoRaWAN.send();
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            LoRaWAN.displayAck();
            LoRaWAN.sleep(loraWanClass);
            break;
        }
        default:
        {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}
