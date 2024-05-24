#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
#define WIFI_SSID "CDT"
#define WIFI_PASSWORD "Otri-cdt2023*"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBtzib-1sThjH5B_NoPTiwtTgO1rhYjeB0"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "marbot-cec3f-default-rtdb.firebaseio.com"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

//some importent variables
String sValue;
bool signupOK = false;
//int vr = D2; // variable pulsador
int sdata = 0; // El valor del pulsador se almacenará en sdata.

int pos0 = 102; //ancho de pulso en cuentas para posición 0°
int pos180 = 512; //ancho de pulso en cuentas para posición 180°

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(27, HIGH);
    delay(300);
    digitalWrite(27, LOW);
  }
}

void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);

  pwm.begin();
  pwm.setPWMFreq(50);

  //pinMode(D5, OUTPUT);
  pinMode(27, OUTPUT);
  // pinMode(vr, INPUT_PULLUP); //Pulsador
  connectToWiFi();

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  }
  else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
    Firebase.begin(&config, &auth); // Reinitialize Firebase connection
    Firebase.reconnectWiFi(true);
  }

  if (Firebase.ready() && signupOK) {
    if (Firebase.RTDB.getString(&fbdo, "/L1")) {
      if (fbdo.dataType() == "string") {
        sValue = fbdo.stringData();
        int a = sValue.toInt();
        Serial.println(a);
        if (a == 1) {
          setServo(2, 108);

        } else {
          setServo(2, 50);

        }
      }
    }
    else {
      Serial.println(fbdo.errorReason());
    }
  }
}

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty = map(angulo, 0, 180, pos0, pos180);
  pwm.setPWM(n_servo, 0, duty);
}
