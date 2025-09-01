// Project: Smart Medibox
// Author: W.A Shehan Piyumantha-220483D

// Include libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHTesp.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <math.h>

// Definition of OLED display parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Definition of pin numbers
#define BUZZER 14
#define LED_1 26
#define LED_2 27
#define PB_CANCEL 16
#define PB_UP 17
#define PB_DOWN 5
#define PB_OK 4
#define DHTPIN 25
#define SERVO_PIN 12
#define LDR_PIN 33

// Definition of MQTT Topics
#define TEMPERATURE_TOPIC "medibox temperature"
#define HUMIDITY_TOPIC "medibox humidity"
#define LIGHT_CURRENT_TOPIC "medibox light intensity"
#define LIGHT_AVG_TOPIC "medibox average light intensity"
#define MOTOR_ANGLE_TOPIC "medibox motor angle"
#define SAMPLING_INTERVAL_TOPIC "medibox sampling interval"
#define SENDING_INTERVAL_TOPIC "medibox sending interval"
#define THETA_OFFSET_TOPIC "medibox theta_offset"
#define GAMMA_TOPIC "medibox gamma"
#define TMED_TOPIC "medibox t_med"

//Definition of NTP Server
#define NTP_SERVER "pool.ntp.org"

//Definition of objects
WiFiUDP ntpUDP;// UDP client for NTP
NTPClient timeClient(ntpUDP, NTP_SERVER);// NTP client
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHTesp dhtSensor;
Servo servo;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

//Definition of Alarm and Snooze parameters
const int NUM_ALARMS = 2;
const int SNOOZE_MINUTES = 5;

//Menu options
enum MenuOptions {
  SET_TIMEZONE,
  SET_ALARM_1,
  SET_ALARM_2,
  VIEW_ALARMS,
  DELETE_ALARM_1,
  DELETE_ALARM_2,
  NUM_MENU_OPTIONS
};

// Global variables for notes
int n_notes = 8;
int C = 262;
int D = 294;
int E = 330;
int F = 349;
int G = 392;
int A = 440;
int B = 494;
int C_H = 523;
int notes[] = {C, D, E, F, G, A, B, C_H};

// Definition of LDR Configuration
unsigned long ldrLastSample = 0;
unsigned long ldrLastSend = 0;
unsigned long tempHumLastSend = 0;
unsigned long servoLastUpdate = 0;
unsigned long dhtLastUpdate = 0; // Timer for DHT updates
int samplingInterval = 5000;    // Default 5 sec
int sendingInterval = 120000;   // Default 2 min
const int MAX_SAMPLES = 24;
float ldrSamples[MAX_SAMPLES] = {0.0};
int sampleIndex = 0;
float lastLdrAvg = 0.0;

// Definition of Servo Equation Parameters
int offset_angle = 30;
float control_factor = 0.75;
float T_med = 30.0;

// Debug flag for printing servo angle
bool printServoAngle = true;

// Definition of Global Variables
int current_hour, current_minute, current_second;
bool alarm_enabled[NUM_ALARMS] = {false, false};
int alarm_hour[NUM_ALARMS] = {0, 0};
int alarm_minute[NUM_ALARMS] = {0, 0};
bool alarm_triggered[NUM_ALARMS] = {false, false};
MenuOptions current_menu_option = SET_TIMEZONE;
float UTC_offset = 5.50;
float new_UTC_offset = UTC_offset;
bool alarm_ringing = false;
bool inMenu = false; // Flag to track if we're in the menu

// Function prototypes
void print_line(String text, int column, int row, int text_size);
void update_current_time();
void check_alarm();
void display_current_data();
void handle_alarm();
void go_to_menu();
void set_timezone();
void set_alarm(int alarm_index);
void view_alarms();
void delete_alarm(int alarm_index);
int wait_for_button_press();
void play_alarm();
void stop_alarm();
void setupMqtt();
void connectToBroker();
void mqttCallback(char* topic, byte* payload, unsigned int length);
int calculateServoAngle(float intensity, float temperature);
float normalizeLdrValue(int rawValue);
void updateLightIntensity();
void execute_menu_action(MenuOptions option);
void updateServoAngle();


// Definition of cached sensor values to avoid blocking delays
float lastTemperature = 0.0;
float lastHumidity = 0.0;
bool sensorError = false;

void setup() {
  //Initializing pins
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(PB_CANCEL, INPUT_PULLUP);
  pinMode(PB_UP, INPUT_PULLUP);
  pinMode(PB_DOWN, INPUT_PULLUP);
  pinMode(PB_OK, INPUT_PULLUP);
  
  //Initializing DHT Sensor
  dhtSensor.setup(DHTPIN, DHTesp::DHT22);

  //Initializing the servo motor
  servo.attach(SERVO_PIN);
  if (!servo.attached()) {
    Serial.println("Servo not attached!");
  }
  servo.write(0);

  Serial.begin(9600);

   //Initialize the OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED initialization failed!");
    for (;;);
  }
  display.display();
  delay(500);

  //Connecting to Wi-Fi
  WiFi.begin("Wokwi-GUEST", "");
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    display.clearDisplay();
    print_line("Connecting to WiFi", 20, 10, 1);
    display.display();
  }
  display.clearDisplay();
  print_line("Connected to WiFi", 20, 10, 1);
  display.display();

  //Connecting to MQTT Broker
  setupMqtt();
  timeClient.begin();
  timeClient.setTimeOffset(UTC_offset * 3600);

  display.clearDisplay();
  print_line("Welcome to", 30, 10, 1);
  print_line("Medibox", 40, 30, 1);
  delay(2000);
  display.clearDisplay();

}

void loop() {
  update_current_time();
  check_alarm();

  // Updating sensors and servo without blocking
  updateLightIntensity();

  // Handling alarm if triggered
  if (alarm_ringing) {
    handle_alarm();
  } else {
    // Only update display if not in menu or alarm
    if (!inMenu) {
      display_current_data();
    }
  }

  if (!mqttClient.connected()) {
    connectToBroker();
  }
  mqttClient.loop();

  // Checking for menu entry
  if (digitalRead(PB_OK) == LOW && !alarm_ringing) {
    delay(50);
    while (digitalRead(PB_OK) == LOW);
    inMenu = true;
    go_to_menu();
    inMenu = false;
  }

  delay(10);
}

// Function to calculate the servo angle using light intensity and temperature data
int calculateServoAngle(float intensity, float temperature) {
  float ts_sec = samplingInterval / 1000.0;
  float tu_sec = sendingInterval / 1000.0;
  float ratio = ts_sec / tu_sec;
  float ln_term = (ratio > 0 && ratio < 1000) ? log(ratio) : 0.0;
  float temp_ratio = temperature / T_med;

  int angle = offset_angle + (180 - offset_angle) * intensity * control_factor * ln_term * temp_ratio;
  return constrain(angle, 0, 180);
}

// Function to update the servo angle using last LDR average and temperature data
void updateServoAngle() {
  if (!isnan(lastTemperature)) {
    int servoAngle = calculateServoAngle(lastLdrAvg, lastTemperature);
    servo.write(servoAngle);

    char angleStr[10];
    dtostrf(servoAngle, 1, 0, angleStr);
    mqttClient.publish(MOTOR_ANGLE_TOPIC, angleStr);

    if (printServoAngle) {
      Serial.print("Calculated Servo Angle: ");
      Serial.println(servoAngle);
    }
  } else {
    Serial.println("Temperature reading failed for servo");
  }
}

// Function to handle MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char messageBuffer[length + 1];
  memcpy(messageBuffer, payload, length);
  messageBuffer[length] = '\0';
  String message = String(messageBuffer);

  bool paramChanged = false;

  if (String(topic) == SAMPLING_INTERVAL_TOPIC) {
    int newInterval = message.toInt();
    if (newInterval >= 1 && newInterval <= 60) {
      samplingInterval = newInterval * 1000;
      paramChanged = true;
    }
  }
  else if (String(topic) == SENDING_INTERVAL_TOPIC) {
    int newInterval = message.toInt();
    if (newInterval >= 60 && newInterval <= 600) {
      sendingInterval = newInterval * 1000;
      paramChanged = true;
    }
  }
  else if (String(topic) == THETA_OFFSET_TOPIC) {
    int newOffset = message.toInt();
    if (newOffset >= 0 && newOffset <= 120) {
      offset_angle = newOffset;
      char paramStr[10];
      dtostrf(offset_angle, 1, 0, paramStr);
      mqttClient.publish("medibox theta_offset/feedback", paramStr);
      paramChanged = true;
    }
  }
  else if (String(topic) == GAMMA_TOPIC) {
    float newGamma = message.toFloat();
    if (newGamma >= 0.0 && newGamma <= 1.0) {
      control_factor = newGamma;
      char paramStr[10];
      dtostrf(control_factor, 1, 2, paramStr);
      mqttClient.publish("medibox gamma/feedback", paramStr);
      paramChanged = true;
    }
  }
  else if (String(topic) == TMED_TOPIC) {
    float newTmed = message.toFloat();
    if (newTmed >= 10.0 && newTmed <= 40.0) {
      T_med = newTmed;
      char paramStr[10];
      dtostrf(T_med, 1, 1, paramStr);
      mqttClient.publish("medibox tmed/feedback", paramStr);
      paramChanged = true;
    }
  }

  if (paramChanged) {
    updateServoAngle();
  }
}

// Function to connect to the MQTT broker
void connectToBroker() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect("MediboxClient-220483D")) {
      mqttClient.subscribe(SAMPLING_INTERVAL_TOPIC);
      mqttClient.subscribe(SENDING_INTERVAL_TOPIC);
      mqttClient.subscribe(THETA_OFFSET_TOPIC);
      mqttClient.subscribe(GAMMA_TOPIC);
      mqttClient.subscribe(TMED_TOPIC);
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5s...");
      delay(5000);
    }
  }
}

// Function to set up MQTT client
void setupMqtt() {
  mqttClient.setServer("broker.hivemq.com", 1883);
  mqttClient.setCallback(mqttCallback);
  connectToBroker();
}

float normalizeLdrValue(int rawValue) {
  return (float)rawValue / 4095.0;
}

// Function to update light intensity and other sensor data
void updateLightIntensity() {
  // Update LDR current value
  if (millis() - ldrLastSample >= samplingInterval) {
    float ldrValue = normalizeLdrValue(analogRead(LDR_PIN));
    ldrSamples[sampleIndex] = ldrValue;
    sampleIndex = (sampleIndex + 1) % MAX_SAMPLES;

    char currentLightStr[10];
    dtostrf(ldrValue, 1, 3, currentLightStr);
    mqttClient.publish(LIGHT_CURRENT_TOPIC, currentLightStr);

    ldrLastSample = millis();
  }

  // Update LDR average values
  if (millis() - ldrLastSend >= sendingInterval) {
    float avg = 0.0;
    for (int i = 0; i < MAX_SAMPLES; i++) {
      avg += ldrSamples[i];
    }
    avg /= MAX_SAMPLES;

    char avgLightStr[10];
    dtostrf(avg, 1, 3, avgLightStr);
    mqttClient.publish(LIGHT_AVG_TOPIC, avgLightStr);

    ldrLastSend = millis();
  }

  // Update temperature and humidity
  if (millis() - tempHumLastSend >= 1000) {
    TempAndHumidity data = dhtSensor.getTempAndHumidity();
    if (!isnan(data.temperature) && !isnan(data.humidity)) {
      lastTemperature = data.temperature;
      lastHumidity = data.humidity;
      sensorError = false;

      char tempStr[10];
      dtostrf(data.temperature, 1, 1, tempStr);
      mqttClient.publish(TEMPERATURE_TOPIC, tempStr);

      char humStr[10];
      dtostrf(data.humidity, 1, 1, humStr);
      mqttClient.publish(HUMIDITY_TOPIC, humStr);
    } else {
      sensorError = true;
      Serial.println("DHT22 reading failed");
    }

    tempHumLastSend = millis();
  }

  // Update servo angle
  if (millis() - servoLastUpdate >= 1000) {
    float avg = 0.0;
    for (int i = 0; i < MAX_SAMPLES; i++) {
      avg += ldrSamples[i];
    }
    avg /= MAX_SAMPLES;
    lastLdrAvg = avg;

    updateServoAngle();

    servoLastUpdate = millis();
  }
}

//Function to display text on the OLED screen
void print_line(String text, int column, int row, int text_size) {
  display.setTextSize(text_size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(column, row);
  display.println(text);
}

//Fetching the current time using NTP server
void update_current_time() {
  timeClient.update();
  current_hour = timeClient.getHours();
  current_minute = timeClient.getMinutes();
  current_second = timeClient.getSeconds();
}

// Function to Check Alarms
void check_alarm() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    if (alarm_enabled[i] && current_hour == alarm_hour[i] &&
        current_minute == alarm_minute[i] && current_second == 0) {
      alarm_triggered[i] = true;
      alarm_ringing = true;
    }
  }
}

// function to display current time, temperature, and humidity
void display_current_data() {
  display.clearDisplay();

  //Displaying the current time on the OLED screen
  String time_str = String(current_hour / 10) + String(current_hour % 10) + ":" +
                    String(current_minute / 10) + String(current_minute % 10) + ":" +
                    String(current_second / 10) + String(current_second % 10);
  print_line(time_str, 20, 0, 2);

  //Displaying the temperature and humidity on the OLED screen
  String temp_str = "Temparature: " + String(lastTemperature, 1) + " C";
  String humidity_str = "Humidity: " + String(lastHumidity, 1) + " %";
  print_line(temp_str, 0, 20, 1);
  print_line(humidity_str, 0, 30, 1);

  //Displaying the light intensity on the OLED screen
  if (sensorError) {
    print_line("Sensor Error!", 0, 40, 1);
    digitalWrite(LED_2, HIGH);
  } else {
    bool warning = false;
    if (lastTemperature > 32) {
      print_line("HIGH TEMPARATURE!", 0, 40, 1);
      digitalWrite(LED_1, HIGH);
      warning = true;
    } else if (lastTemperature < 24) {
      print_line("LOW TEMPARATURE!", 0, 40, 1);
      digitalWrite(LED_1, HIGH);
      warning = true;
    } else {
      print_line("TEMPARATURE OK", 0, 40, 1);
      digitalWrite(LED_1, LOW);
    }

    if (lastHumidity > 80) {
      print_line("HIGH HUMIDITY!", 0, 50, 1);
      digitalWrite(LED_2, HIGH);
      warning = true;
    } else if (lastHumidity < 65) {
      print_line("LOW HUMIDITY!", 0, 50, 1);
      digitalWrite(LED_2, HIGH);
      warning = true;
    } else {
      print_line("HUMIDITY OK", 0, 50, 1);
      digitalWrite(LED_1, LOW);
    }

    digitalWrite(LED_2, warning ? HIGH : LOW);
  }

  display.display();
}

// Function to handle alarm actions
void handle_alarm() {
  for (int i = 0; i < NUM_ALARMS; i++) {
    if (alarm_triggered[i]) {
      display.clearDisplay();
      print_line("MEDICINE TIME!", 10, 10, 1);
      print_line("OK: Stop", 0, 30, 1);
      print_line("CANCEL: Snooze", 0, 40, 1);
      display.display();

      //Playing the alarm  and turning on the LED
      play_alarm();
      digitalWrite(LED_1, HIGH);

      int button_press = wait_for_button_press();

      if (button_press == PB_OK) {
        alarm_triggered[i] = false;
        alarm_ringing = false;
        stop_alarm();
        digitalWrite(LED_1, LOW);
      } else if (button_press == PB_CANCEL) {
        alarm_triggered[i] = false;
        alarm_ringing = false;
        stop_alarm();
        digitalWrite(LED_1, LOW);

        // Snooze the alarm
        int new_minute = alarm_minute[i] + SNOOZE_MINUTES;
        if (new_minute >= 60) {
          alarm_hour[i] = (alarm_hour[i] + 1) % 24;
          alarm_minute[i] = new_minute - 60;
        } else {
          alarm_minute[i] = new_minute;
        }
      }
    } else {
      stop_alarm();
      digitalWrite(LED_1, LOW);
    }
  }
}

// Function to go to the menu
void go_to_menu() {
  while (true) {
    display.clearDisplay();
    //print_line("Main Menu", 20, 0, 2);

    switch (current_menu_option) {
      case SET_TIMEZONE:
        print_line("1. Set Timezone", 0, 20, 1);
        break;
      case SET_ALARM_1:
        print_line("2. Set Alarm 1", 0, 20, 1);
        break;
      case SET_ALARM_2:
        print_line("3. Set Alarm 2", 0, 20, 1);
        break;
      case VIEW_ALARMS:
        print_line("4. View Alarms", 0, 20, 1);
        break;
      case DELETE_ALARM_1:
        print_line("5. Delete Alarm 1", 0, 20, 1);
        break;
      case DELETE_ALARM_2:
        print_line("6. Delete Alarm 2", 0, 20, 1);
        break;
    }
    display.display();

    // Display the current menu option
    int button_press = wait_for_button_press();
    switch (button_press) {
      case PB_UP:
        current_menu_option = (MenuOptions)(((int)current_menu_option + 1) % NUM_MENU_OPTIONS);
        break;
      case PB_DOWN:
        current_menu_option = (MenuOptions)(((int)current_menu_option - 1 + NUM_MENU_OPTIONS) % NUM_MENU_OPTIONS);
        break;
      case PB_OK:
        execute_menu_action(current_menu_option);
        return;
      case PB_CANCEL:
        return;
    }
  }
}

// Function to Set Timezone by user input
void set_timezone() {
  while (true) {
    display.clearDisplay();
    print_line("Set UTC Offset", 20, 10, 1);
    print_line(String(new_UTC_offset, 1), 50, 30, 2);
    display.display();

    int button_press = wait_for_button_press();
    if (button_press == PB_UP) {
      new_UTC_offset = fmin(14.0, new_UTC_offset + 0.5);
    } else if (button_press == PB_DOWN) {
      new_UTC_offset = fmax(-12.0, new_UTC_offset - 0.5);
    } else if (button_press == PB_OK) {
      UTC_offset = new_UTC_offset;
      timeClient.setTimeOffset(UTC_offset * 3600);
      display.clearDisplay();
      print_line("Timezone Set!", 20, 20, 1);
      display.display();
      delay(1000);
      break;
    }
  }
}

//Function to Set Alarm by user input
void set_alarm(int alarm_index) {
  int temp_hour = alarm_hour[alarm_index];
  int temp_minute = alarm_minute[alarm_index];

  while (true) {
    display.clearDisplay();
    print_line("Set Hour:", 20, 10, 1);
    print_line(String(temp_hour), 50, 30, 2);
    display.display();

    int button_press = wait_for_button_press();
    if (button_press == PB_UP) temp_hour = (temp_hour + 1) % 24;
    else if (button_press == PB_DOWN) temp_hour = (temp_hour + 23) % 24;
    else if (button_press == PB_OK) break;
  }

  while (true) {
    display.clearDisplay();
    print_line("Set Minute:", 20, 10, 1);
    print_line(String(temp_minute), 50, 30, 2);
    display.display();

    int button_press = wait_for_button_press();
    if (button_press == PB_UP) temp_minute = (temp_minute + 1) % 60;
    else if (button_press == PB_DOWN) temp_minute = (temp_minute + 59) % 60;
    else if (button_press == PB_OK) break;
  }

  alarm_hour[alarm_index] = temp_hour;
  alarm_minute[alarm_index] = temp_minute;
  alarm_enabled[alarm_index] = true;

  display.clearDisplay();
  print_line("Alarm Set!", 20, 20, 1);
  display.display();
  delay(1000);
}

//Function to view alarms through the menu
void view_alarms() {
  display.clearDisplay();
  print_line("Active Alarms:", 0, 0, 1);
  for (int i = 0; i < NUM_ALARMS; i++) {
    String alarm_status = "Alarm " + String(i+1) + ": ";
    if (alarm_enabled[i]) {
      alarm_status += String(alarm_hour[i]) + ":" + String(alarm_minute[i]);
    } else {
      alarm_status += "Disabled";
    }
    print_line(alarm_status, 0, 10*(i+2), 1);
  }
  display.display();
  delay(3000);
}

void delete_alarm(int alarm_index) {
  alarm_enabled[alarm_index] = false;
  display.clearDisplay();
  print_line("Alarm " + String(alarm_index+1) + " Deleted", 0, 20, 1);
  display.display();
  delay(1000);
}

// Function to execute menu actions based on user input
void execute_menu_action(MenuOptions option) {
  switch (option) {
    case SET_TIMEZONE: set_timezone(); break;
    case SET_ALARM_1: set_alarm(0); break;
    case SET_ALARM_2: set_alarm(1); break;
    case VIEW_ALARMS: view_alarms(); break;
    case DELETE_ALARM_1: delete_alarm(0); break;
    case DELETE_ALARM_2: delete_alarm(1); break;
  }
}

// Function to wait for button press and return the button pressed
int wait_for_button_press() {
  unsigned long startTime = millis();
  while (millis() - startTime < 30000) { // Timeout after 30 seconds
    if (digitalRead(PB_UP) == LOW) {
      delay(50);
      while (digitalRead(PB_UP) == LOW);
      return PB_UP;
    }
    if (digitalRead(PB_DOWN) == LOW) {
      delay(50);
      while (digitalRead(PB_DOWN) == LOW);
      return PB_DOWN;
    }
    if (digitalRead(PB_OK) == LOW) {
      delay(50);
      while (digitalRead(PB_OK) == LOW);
      return PB_OK;
    }
    if (digitalRead(PB_CANCEL) == LOW) {
      delay(50);
      while (digitalRead(PB_CANCEL) == LOW);
      return PB_CANCEL;
    }
    delay(10);
  }
  return PB_CANCEL; // Timeout returns as cancel
}

//Function to play the alarm
void play_alarm() {
  tone(BUZZER, 1000);
  delay(500);
  for (int i = 0; i < n_notes; i++) {
    tone(BUZZER, notes[i]);
    delay(500);
  }
}

void stop_alarm() {
  noTone(BUZZER);
}