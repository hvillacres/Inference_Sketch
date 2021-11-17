//Libraries
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"                     // Librerias I2C para controlar el mpu6050
#include "MPU6050.h"
#include <Adafruit_GFX.h>               //libreria necesaria para inicializar oled, graficos
#include <Adafruit_SSD1306.h>           //libreria necesaria para inicializar oled
#include <Fonts/FreeMonoBold9pt7b.h>    //Fuente de Letra de bienvenida para el oled
#include <Fonts/FreeSans9pt7b.h>        //Fuente de Letra de Mensajes
#include "images.h"                     //Imagenes a mostrar en el Oled
#include <WiFiUdp.h>                    //Udp.cpp: Biblioteca para enviar / recibir paquetes UDP
#include <PubSubClient.h>               //Permite la Comunicación con Hassio a travez de mqtt
#include "SoundData.h"
#include "XT_DAC_Audio.h"

///////////////////////////////////// Variables globales /////////////////////////////////////

const int pinStart = 4;         //Button D4
boolean flag = false;           //Bandera, utilizada para determinar si se presionó el boton
unsigned long debounce = 0;     // Tiempo del rebote.
unsigned long previo, actual;   // Tiempo de ejecución
unsigned long lastTime = 0, sampleTime = 100;

///////////////////////////////////// Archivos de Audio /////////////////////////////////////

//Create an object of type XT_Wav_Class that is used by the dac audio class (below), passing wav data as parameter.
XT_Wav_Class audio_G1(a_dia);
XT_Wav_Class audio_G2(a_tarde);
XT_Wav_Class audio_G3(a_noche);
XT_Wav_Class audio_G4(a_cts);
XT_Wav_Class audio_G5(a_iluminar);
XT_Wav_Class audio_G5_1(a_apagar);
XT_Wav_Class audio_G6(a_conectar);
XT_Wav_Class audio_G6_1(a_desconectar);
XT_Wav_Class audio_G7(a_hora);
XT_Wav_Class audio_G8(a_direccion);
XT_Wav_Class audio_G9(a_nombre);
XT_Wav_Class audio_G10(a_edad);

XT_DAC_Audio_Class DacAudio(25,0);    // Create the main player class object. 
                                      // Use GPIO 25, one of the 2 DAC pins and timer 0

///////////////////////////////////// Initialize SSD1306 /////////////////////////////////////

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int mensaje = 0;

///////////////////////////////////////////   WIFI   /////////////////////////////////////////

// WiFi Network Credentials
const char *ssid =  "ssid";   // name of your WiFi network
const char *password =  "password"; // password of the WiFi network

////////////////////////// Inicializar MPU6050 //////////////////////////

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

// Instancia objeto de la clase MPU6050
MPU6050 sensor;

// Valores RAW (sin procesar) del giroscopio en los ejes x,y,z
const int timeData = 3; // Tiempo en recolectar datos en segundos
int16_t accelX = 0, accelY = 0, accelZ = 0;
int16_t gyroX = 0, gyroY = 0, gyroZ = 0;

//////////////////////////////////////////   MQTT   //////////////////////////////////////////

// Home Assistant Credentials
const char *HA_USER = "TTgo_Camera";
const char *HA_PASS = "Leonel/*123";

// MQTT Network
IPAddress broker(192,168,100,178); // IP address of your MQTT broker eg. 192.168.1.50
//Bombillo
const char *ID_LIGHT = "Example_Switch";  // Name of our device, must be unique
const char *TOPIC_LIGHT = "room/light";  // Topic to subcribe to
const char *MESSAGE = "toggle";   // Message to publish to our topic
//Tomacorriente
const char *ID_PLUG = "Example_Switch_Plug";  // Name of our device, must be unique
const char *TOPIC_PLUG = "room/plug";  // Topic to subcribe to
WiFiClient wclient;

int checkLight = 0;     //Verifica si el bombillo está encendido.
int checkPlug = 0;     //Verifica si el tomacorriente está encendido.

PubSubClient client(wclient); // Setup MQTT client

/////////////////////////////////// CARACTERISTICAS CNN ///////////////////////////////////

// Caracteristicas Acelerómetro
double mavXAxis = 0.0,mavYAxis = 0.0,mavZAxis = 0.0;
double rmsXAxis = 0.0,rmsYAxis = 0.0,rmsZAxis = 0.0;
double wlXAxis = 0.0, wlYAxis = 0.0, wlZAxis = 0.0;
double wlaXAxis = 0.0, wlaYAxis = 0.0, wlaZAxis = 0.0;

// Caracteristicas Giroscopio
double GmavXAxis = 0.0, GmavYAxis = 0.0, GmavZAxis = 0.0;
double GrmsXAxis = 0.0, GrmsYAxis = 0.0, GrmsZAxis = 0.0;
double GwlXAxis = 0.0, GwlYAxis = 0.0, GwlZAxis = 0.0;
double GwlaXAxis = 0.0, GwlaYAxis = 0.0, GwlaZAxis = 0.0;


///////////////////////////////// Variables Red Neuronal /////////////////////////////////
double a0[18];
double W1[6][18] = {{-0.226,-0.357,-0.492,-0.222,0.232,-0.049,-0.4,-0.625,-0.096,0.092,1.263,0.375,1.282,1.588,0.44,-1.074,-0.46,-0.62},{-0.042,-0.373,0.201,0.402,-0.296,0.489,-0.547,0.202,-0.897,-1.067,-0.627,-1.019,-1.059,0.278,-0.587,-0.48,-1.578,-1.477},{-0.283,-1.228,1.417,-0.976,-0.776,1.265,0.316,0.773,0.957,0.694,-0.363,0.778,0.572,-0.691,1.204,-0.049,0.111,0.074},{1.732,-1.009,-0.944,0.776,-0.641,-1.557,0.309,-0.069,-0.408,0.021,-0.454,-0.07,-0.61,-0.617,-0.645,-0.126,0.799,0.355},{0.231,0.887,-0.333,0.195,0.083,0.225,-0.602,0.611,0.891,1.162,-0.353,-0.656,0.903,0.317,-1.183,1.822,-0.57,-0.504},{-0.36,0.699,-0.78,0.156,0.197,-0.377,1.255,0.309,0.569,-1.308,-0.809,0.026,-0.994,0.159,0.055,-0.183,0.733,1.124}};
double a1[6];
double W2[10][6] = {{-0.194,0.441,-2.725,-1.738,1.653,-1.122},{0.559,-0.458,-0.138,-1.006,0.551,-1.643},{-0.111,-1.125,1.585,-1.062,-0.747,-1.205},{-1.77,-0.161,-0.803,0.645,1.159,0.143},{-0.918,1.201,-1.089,-2.057,-1.139,-0.934},{0.607,0.516,-1.711,0.322,-1.649,-0.691},{-2.32,0.088,1.349,-0.548,0.449,-0.697},{-0.41,-0.705,-0.103,1.591,-0.34,-2.249},{-2.044,0.108,-1.803,0.601,-1.85,0.894},{0.753,-0.747,-0.814,-0.449,-0.901,0.862}};
double a2[10]; 
double b1[6]= {1.675,1.061,0.922,0.936,1.757,0.866};
double b2[10]= {1.154,0.304,-0.199,-0.767,-0.468,0.994,-0.47,-0.43,-0.087,-0.393};
double aux = 0.0;
//////////////////////////////////////////////////////////////////

///////////////////////////////// Preprocesamiento Red Neuronal /////////////////////////////////
double mean[18]={6.204,3.942,4.841,6.872,4.77,5.731,64.289,65.907,65.375,67.517,49.43,62.098,88.531,70.97,84.177,1691.906,1372.43,1480.691};
double dstd[18]={2.14,1.216,2.267,2.151,1.249,2.378,34.667,33.347,29.153,31.827,20.758,26.525,35.52,27.941,30.315,1134.64,575.559,816.238};
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////// Mensaje de bienvenida e Inicialización de la fuente ///////////////////
void init_welcome(){
  display.setFont(&FreeMonoBold9pt7b);
  display.clearDisplay();
  display.drawRect(0, 17, 128, 26, WHITE);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(7,34);             
  display.println("BIENVENIDO");
  display.display();
  delay(1000);
}


////////////////////////////////// Connect to WiFi network //////////////////////////////////
void setup_wifi() {
  Serial.print("\nConnecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password); // Connect to network
  delay(1000);
    while (WiFi.status() != WL_CONNECTED) { // Wait for connection
      delay(500);
      Serial.print(".");
    }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
}


///////////////////////////////// Reconnect to client - MQTT ////////////////////////////////
void reconnect() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,26);
  display.println("WIFI...");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,45);
  display.println("MQTT...");
  display.display();
  /*delay(1000);*/

  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID_LIGHT,HA_USER,HA_PASS)) {
      Serial.println("connected");
      Serial.print("Publishing to: ");
      Serial.println(TOPIC_LIGHT);
      Serial.println('\n');

    } else {
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    
    if (client.connect(ID_PLUG,HA_USER,HA_PASS)) {
      Serial.println("connected");
      Serial.print("Publishing to: ");
      Serial.println(TOPIC_PLUG);
      Serial.println('\n');

    } else {
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
  display.clearDisplay();
  display.setCursor(20,26);
  display.println("WIFI: OK");
  display.display();
  display.setCursor(20,45);
  display.println("MQTT: OK");
  display.display();
}


//////////////////////////////// Inicializo los pines - MQTT ////////////////////////////////
void setup_mqtt(){
  pinMode(pinStart,INPUT);  // Configure SWITCH_Pin as an input
  digitalWrite(pinStart,HIGH);  // enable pull-up resistor (active low)
  delay(100);
}


/////////////////////////////// Extracción de Características ///////////////////////////////
void featuresExtraction() {
    
  int sizeSample = timeData*10;
  
  // Caracteristicas Acelerómetro
  mavXAxis = 0.0;
  mavYAxis = 0.0;
  mavZAxis = 0.0;

  rmsXAxis = 0.0;
  rmsYAxis = 0.0;
  rmsZAxis = 0.0;

  wlXAxis = 0.0;
  wlYAxis = 0.0;
  wlZAxis = 0.0;

  // Caracteristicas Giroscopio
  GmavXAxis = 0.0;
  GmavYAxis = 0.0;
  GmavZAxis = 0.0;

  GrmsXAxis = 0.0;
  GrmsYAxis = 0.0;
  GrmsZAxis = 0.0;

  GwlXAxis = 0.0;
  GwlYAxis = 0.0;
  GwlZAxis = 0.0;

  for (int k = 0; k<sizeSample;k++)
  { 
    sensor.getAcceleration(&accelX, &accelY, &accelZ);
    sensor.getRotation(&gyroX, &gyroY, &gyroZ);
   
    double ax_m_s2 = accelX * (9.81/16384.0);
    double ay_m_s2 = accelY * (9.81/16384.0);
    double az_m_s2 = accelZ * (9.81/16384.0);

    float gx_deg_s = gyroX * (250.0/32768.0);
    float gy_deg_s = gyroY * (250.0/32768.0);
    float gz_deg_s = gyroZ * (250.0/32768.0);

    ////////////////// Caracteristicas Acelerómetro //////////////////

    //Valor absoluto medio (MAV)
    mavXAxis = mavXAxis + abs(ax_m_s2);
    mavYAxis = mavYAxis + abs(ay_m_s2);
    mavZAxis = mavZAxis + abs(az_m_s2);

    //Valor eficaz (RMS)
    rmsXAxis = rmsXAxis + ax_m_s2*ax_m_s2;
    rmsYAxis = rmsYAxis + ay_m_s2*ay_m_s2;
    rmsZAxis = rmsZAxis + az_m_s2*az_m_s2;

    //Longitud de forma de onda (WL)
    wlXAxis = wlXAxis + abs(ax_m_s2 - wlaXAxis);
    wlYAxis = wlYAxis + abs(ay_m_s2 - wlaYAxis);
    wlZAxis = wlZAxis + abs(az_m_s2 - wlaZAxis);

    wlaXAxis = ax_m_s2;
    wlaYAxis = ay_m_s2;
    wlaZAxis = az_m_s2;

    ////////////////// Caracteristicas Giroscopio //////////////////
    //Valor absoluto medio (MAV)
    GmavXAxis = GmavXAxis + abs(gx_deg_s);
    GmavYAxis = GmavYAxis + abs(gy_deg_s);
    GmavZAxis = GmavZAxis + abs(gz_deg_s);

    ///Valor eficaz (RMS)
    GrmsXAxis = GrmsXAxis + gx_deg_s*gx_deg_s;
    GrmsYAxis = GrmsYAxis + gy_deg_s*gy_deg_s;
    GrmsZAxis = GrmsZAxis + gz_deg_s*gz_deg_s;

    //Longitud de forma de onda (WL)
    GwlXAxis = GwlXAxis + abs(gx_deg_s - GwlaXAxis);
    GwlYAxis = GwlYAxis + abs(gy_deg_s - GwlaYAxis);
    GwlZAxis = GwlZAxis + abs(gz_deg_s - GwlaZAxis);

    GwlaXAxis = gx_deg_s;
    GwlaYAxis = gy_deg_s;
    GwlaZAxis = gz_deg_s;
    
    delay(100);
  }

  ////////////////// Caracteristicas Acelerómetro //////////////////
  mavXAxis = mavXAxis/(double)sizeSample;
  mavYAxis = mavYAxis/(double)sizeSample;
  mavZAxis = mavZAxis/(double)sizeSample;
  
  rmsXAxis = sqrt(rmsXAxis/(double)sizeSample);
  rmsYAxis = sqrt(rmsYAxis/(double)sizeSample);
  rmsZAxis = sqrt(rmsZAxis/(double)sizeSample);

  ////////////////// Caracteristicas Giroscopio //////////////////
  GmavXAxis = GmavXAxis/(double)sizeSample;
  GmavYAxis = GmavYAxis/(double)sizeSample;
  GmavZAxis = GmavZAxis/(double)sizeSample;
  
  GrmsXAxis = sqrt(GrmsXAxis/(double)sizeSample);
  GrmsYAxis = sqrt(GrmsYAxis/(double)sizeSample);
  GrmsZAxis = sqrt(GrmsZAxis/(double)sizeSample);
}


////////////////////////////////// Funciones de acticación //////////////////////////////////
double relu(double n) {
  if(n>=0) return n; else if (n<0) return 0;
}


double sigmoid(double n) {
  return 1.0/(1.0 + exp(-n));
}


///////////////////////////////// Función de normalización /////////////////////////////////
double dataNormalized(double inputData,double mean,double desvStandar) {
  double valueNorm;
  valueNorm = (inputData-mean)/desvStandar;
  return valueNorm;
}

////////////////////////////// Imprime el mensaje en el Display //////////////////////////////
void mensajeOled(int mensaje){
  display.clearDisplay();
  /*display.setFont();*/
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  /*display.setCursor(0,32);*/
  display.setCursor(10,30);
  if(mensaje == 1){
    display.cp437(true);      //Activar página de código 437
    display.write(173);       // Escribir el carácter ¡
    display.println("!Buenos");
    display.setCursor(20,49);
    display.println("dias!");
  }
  if(mensaje == 2){
    display.cp437(true);      //Activar página de código 437
    display.write(173);       // Escribir el carácter ¡
    display.println("!Buenas");
    display.setCursor(15,49);
    display.println("tardes!");
  }
  if(mensaje == 3){
    display.cp437(true);      //Activar página de código 437
    display.write(173);       // Escribir el carácter ¡
    display.println("!Buenas");
    display.setCursor(15,49);
    display.println("noches!");
  }
  if(mensaje == 4){
    display.cp437(true);      //Activar página de código 437
    display.write(168);       // Escribir el carácter ?
    display.println("¿Como te");
    display.setCursor(13,49);
    display.println("sientes?");
  }
  if(mensaje == 5){
    display.clearDisplay();
    display.drawBitmap(0, 0, foco_on, 128, 64, 1);  // Draw bitmap on the screen
  }
  if(mensaje == 55){
    display.clearDisplay();
    display.drawBitmap(0, 0, foco_off, 128, 64, 1);  // Draw bitmap on the screen
  }
  if(mensaje == 6){
    display.clearDisplay();
    display.drawBitmap(0, 0, rayo_on, 128, 64, 1);  // Draw bitmap on the screen
  }
  if(mensaje == 66){
    display.clearDisplay();
    display.drawBitmap(0, 0, rayo_off, 128, 64, 1);  // Draw bitmap on the screen
  }
  if(mensaje == 7){
    display.cp437(true);      //Activar página de código 437
    display.write(168);       // Escribir el carácter ?
    display.println("¿Que hora es?");
  }
  if(mensaje == 8){
    display.cp437(true);      //Activar página de código 437
    display.write(168);       // Escribir el carácter ?
    display.println("¿Como llego a");
    display.setCursor(0,49);
    display.println("esta direccion?");
  }
  if(mensaje == 9){
    display.cp437(true);      //Activar página de código 437
    display.write(168);       // Escribir el carácter ?
    display.println("¿Como te");
    display.setCursor(13,49);
    display.println("llamas?");
  }
  if(mensaje == 10){
    display.cp437(true);      //Activar página de código 437
    display.write(168);       // Escribir el carácter ?
    display.println("¿Que edad");
    display.setCursor(16,49);
    display.println("tienes?");
  }
  display.display();
  mensaje = 0;
  // Mover texto de izquierda a derecha
  display.startscrollright(0x00, 0x0F);
}

//////////////////////////// Presionar el boton, activar bandera ////////////////////////////
void pulse() {
  if(!digitalRead(pinStart) && (millis()-debounce > 500))
  {
    debounce = millis();
    flag = true;
   } 
}


void setup() {
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }
  delay(2000);
  
  setup_wifi(); //Connect to network
  init_welcome(); //Init MQTT
  setup_mqtt();   //inicializo MQtt
  
  client.setServer(broker, 1883);

  pinMode(pinStart,INPUT_PULLUP); //Configuramos el modo de trabajo del boton, entrada

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  //Serial.println("Iniciando MPU6050");
  if (sensor.testConnection()) delay(500); //Serial.println("Sensor iniciado correctamente"); 
  else delay(500); //Serial.println("Error al iniciar el sensor");

}


void loop() {

  DacAudio.FillBuffer();                // Fill the sound buffer with data

  //Reconexión con el servidor
  if (!client.connected()) { // Reconnect if connection is lost
    reconnect();
  }
  client.loop();

  //Condición si se presionó el botón, este retorna un true, en la bandera
  if(!digitalRead(pinStart)) {
      pulse();
  }
  
  //Si la bandera es true, sucede el ingreso de datos
  if(flag) {
    display.stopscroll();
    display.setFont();
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(62,25);
    display.println("2");
    display.display();
    delay(1000);
    display.clearDisplay();
    display.setCursor(62,25);
    display.println("1");
    display.display();
    delay(1000);
    display.setFont(&FreeSans9pt7b);
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0,32);
    display.println("¡Empiece Ahora!");
    display.display();
    delay(100);
    
    //previo = millis();
    //Extracción de datos
    featuresExtraction();
    previo = millis();

    //Normalizar las características y almacenar en el vector de entrada
    ////////////////// Caracteristicas Acelerómetro //////////////////
    a0[0] = dataNormalized(mavXAxis,mean[0],dstd[0]);
    a0[1] = dataNormalized(mavYAxis,mean[1],dstd[1]);
    a0[2] = dataNormalized(mavZAxis,mean[2],dstd[2]);

    a0[3] = dataNormalized(rmsXAxis,mean[3],dstd[3]);
    a0[4] = dataNormalized(rmsYAxis,mean[4],dstd[4]);
    a0[5] = dataNormalized(rmsZAxis,mean[5],dstd[5]);

    a0[6] = dataNormalized(wlXAxis,mean[6],dstd[6]);
    a0[7] = dataNormalized(wlYAxis,mean[7],dstd[7]);
    a0[8] = dataNormalized(wlZAxis,mean[8],dstd[8]);

    ////////////////// Caracteristicas Giroscopio //////////////////
    a0[9] = dataNormalized(GmavXAxis,mean[9],dstd[9]);
    a0[10] = dataNormalized(GmavYAxis,mean[10],dstd[10]);
    a0[11] = dataNormalized(GmavZAxis,mean[11],dstd[11]);

    a0[12] = dataNormalized(GrmsXAxis,mean[12],dstd[12]);
    a0[13] = dataNormalized(GrmsYAxis,mean[13],dstd[13]);
    a0[14] = dataNormalized(GrmsZAxis,mean[14],dstd[14]);

    a0[15] = dataNormalized(GwlXAxis,mean[15],dstd[15]);
    a0[16] = dataNormalized(GwlYAxis,mean[16],dstd[16]);
    a0[17] = dataNormalized(GwlZAxis,mean[17],dstd[17]);

    ///////////////////////////////// Estructura Red Neuronal /////////////////////////////////
    for(int i = 0 ; i<6; i++ ) {aux=0.0;for(int j = 0 ; j <18 ; j++ ) { aux=aux+W1[i][j]*a0[j];} a1[i]=relu(aux+b1[i]);}
    double aux1 = 0;
    for(int i = 0 ; i<10; i++ ) {aux=0.0;for(int j = 0 ; j <6 ; j++ ){ aux=aux+W2[i][j]*a1[j];} a2[i]=(aux+b2[i]);aux1=aux1+exp(a2[i]);}
    double minimo = 0.0;
    int classes = 0;
    for(int i = 0;  i<10; i++){a2[i] = exp(a2[i])/aux1;if(a2[i]>minimo){minimo=a2[i];classes=i;}}
    //////////////////////////////////////////////////////////////////////////////////////////

    switch (classes) {
      case 0:
        Serial.println("Good Morning!"); //Serial.println("¡Buenos días!");  
        mensaje = 1;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G1);
        break;
      case 1:
        Serial.println("Good afternoon!"); //Serial.println("¡Buenas tardes!");
        mensaje = 2;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G2);
        break;
      case 2:
        Serial.println("Good night!"); //Serial.println("¡Buenas noches!");
        mensaje = 3;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G3);
        break;
      case 3:
        Serial.println("How do you feel?"); //Serial.println("¿Cómo te sientes?");
        mensaje = 4;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G4);
        break;
      case 4:
        if(checkLight == 1){
          Serial.println("Light off!"); //Serial.println("¡Luz apagada!");
          client.publish(TOPIC_LIGHT, MESSAGE);
          Serial.println((String)TOPIC_LIGHT + " => toggle");
          checkLight = 0;     //Asigna el valor 0, indicando que este está apagado
          mensaje = 55;
          mensajeOled(mensaje);
          DacAudio.Play(&audio_G5_1);
        }else{
          Serial.println("Light on!"); //Serial.println("¡Luz encendida!"); 
          client.publish(TOPIC_LIGHT, MESSAGE);
          Serial.println((String)TOPIC_LIGHT + " => toggle");
          checkLight = 1;     //Asigna el valor 1, indicando que este está encendido
          mensaje = 5;
          mensajeOled(mensaje);
          DacAudio.Play(&audio_G5);
        }
        break;
      case 5:
        if(checkPlug == 1){
          Serial.println("plug disabled!"); //Serial.println("¡Tomacorriente desactivado!");
          client.publish(TOPIC_PLUG, MESSAGE);
          Serial.println((String)TOPIC_PLUG + " => toggle");
          checkPlug = 0;    //Asigna el valor 0, indicando que este está apagado
          mensaje = 66;     //Asigna el valor 66, indicando q imprima el mensaje 66
          mensajeOled(mensaje);   //Llama al método para imprimir el mensaje en el display
          DacAudio.Play(&audio_G6_1);   //Menciona el audio a reproducir.
        }else{
          Serial.println("plug activated!"); //Serial.println("¡Tomacorriente activado!");
          client.publish(TOPIC_PLUG, MESSAGE);
          Serial.println((String)TOPIC_PLUG + " => toggle");
          checkPlug = 1;    //Asigna el valor 1, indicando que este está encendido
          mensaje = 6;
          mensajeOled(mensaje);
          DacAudio.Play(&audio_G6);
        }
        break;
      case 6:
        Serial.println("What time is it?"); //Serial.println("¿Qué hora es?");
        mensaje = 7;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G7);
        break;
      case 7:
        Serial.println("How do I get to this address?"); //Serial.println("¿Cómo llego a esta dirección?");
        mensaje = 8;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G8);
        break;
      case 8:
        Serial.println("What is your name?"); //Serial.println("¿Cómo te llamas?");
        mensaje = 9;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G9);
        break;
      case 9:
        Serial.println("How old are you?"); //Serial.println("¿Cuántos años tienes?");
        mensaje = 10;
        mensajeOled(mensaje);
        DacAudio.Play(&audio_G10);
        break;
    }
    //La bandera regresa a false
    flag = false;
    actual = millis();
    Serial.print("Executed in ");
    Serial.print(actual - previo);
    Serial.println(" ms");
    Serial.println(" ");
  } 
}