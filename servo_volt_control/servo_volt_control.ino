#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

AsyncWebServer server(80);

const char* ssid = "ESP32-WebSerial"; //WiFi AP SSID 
const char* password = "12345678";    //WiFi Password

double Setpoint, pidInput, pidOutput;
double Kp = 2;                        //Nilai Kp
double Ki = 5;                        //Nilai Ki
double Kd = 1;                        //Nilai Kd
PID pid(&pidInput, &pidOutput, &Setpoint, Kp, Ki, Kd, DIRECT); 

#define servoPin 15                   //Pin Servo
#define pressurePin 14                //Pin Pressure Sensor
#define voltPin 34                    //Pin Output Tegangan
Servo servo;

float maxVoltage = 3.3;     //tegangan maksimum yang bisa dibaca esp32
float maxAnaRead = 4095.0;  //nilai analog maksimum 12-bit
float calVolt = 1.0;        //kalibrasi nilai tegangan
int anaRead; 

float minPressure = 0.00;    //tekanan minimum dari sensor
float maxPressure = 12.00;  //tekanan maksimum dari sensor
float calPressure = 1.00;    //kalibrasi nilai tekanan

float minAngle = 0.00;       //batas minimum operasi servo
float maxAngle = 90.00;     //batas maksimum operasi servo
float angle;                //sudut servo

float getValSensor(int analogPin){
  anaRead = analogRead(analogPin);

  float voltResult = ((anaRead*maxVoltage)/maxAnaRead)*calVolt;
  return voltResult;
}

//fungsi mencari nilai tekanan(bar) dari tegangan sensor transducer yang dibaca
float getPressure(float vRes){
  float pressurePascal = (3.0*(vRes-0.47))*1000000.0*calPressure;
  float pressureBar = pressurePascal/10e5;
  return pressureBar;
}

//fungsi mengoperasikan servo berdasarkan tegangan output sensor yang dibaca
void controlServo(float volt){
  angle = (volt/maxVoltage)*maxAngle;
  servo.write(angle);
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  WebSerial.begin(&server);

  server.begin();
  servo.attach(servoPin);
  Setpoint = 100;
  pid.SetMode(AUTOMATIC);
}

void loop() {
  WebSerial.print("Tuning PID > ");
  WebSerial.println("Kp = " +String(pid.GetKp()) + " || " + "Ki = " +String(pid.GetKi()) + " || " + "Kd = " +String(pid.GetKd()));
  Serial.print("Tuning PID > ");
  Serial.println("Kp = " +String(pid.GetKp()) + " || " + "Ki = " +String(pid.GetKi()) + " || " + "Kd = " +String(pid.GetKd()));

  
  unsigned long millisAwal = millis();                        //mencatat starting point time
  float teganganOutput = getValSensor(voltPin);               //mengambil data output tegangan generator
  int analogVolt = anaRead;                                   //mencatat nilai analog yang terbaca pada output tegangan
  pidInput = teganganOutput;                                  //memasukkan nilai tegangan output pada variable pid
  pid.Compute();                                              //menghitung nilai output berdasarkan nilai pid
  controlServo(pidOutput);                                    //mengatur servo berdasarkan output pid
  unsigned long millisAkhir = millis();                       //mencatat end point time
  float teganganPressure = getValSensor(pressurePin);         //mencatat tegangan output pressure sensor
  int analogPressure = anaRead;                               //mencatat nilai analog yang terbaca pada output pressure sensor
  float pressureOutput = getPressure(teganganPressure);       //menghitung nilai output pressure dalam satuan bar
  
  
  WebSerial.println("Millis input = " + String(millisAwal) + "ms >< Millis output = " + String(millisAkhir) + "ms");
  WebSerial.println("Tekanan Output Kompressor = "+String(pressureOutput, 2) + "v || Nilai Analog = " + String(analogPressure));
  WebSerial.println("Tegangan Output Turbin = "+String(teganganOutput, 2) + "v || Nilai Analog = " + String(analogVolt));
  WebSerial.println("Sudut Bukaan Servo = "+String(angle, 2));
  WebSerial.println("-----------------------------------------------------");
  Serial.println("Millis input = " + String(millisAwal) + "ms >< Millis output = " + String(millisAkhir) + "ms");
  Serial.println("Tekanan Output Kompressor = "+String(pressureOutput, 2) + "bar || Nilai Analog = " + String(analogPressure));
  Serial.println("Tegangan Output Turbin = "+String(teganganOutput, 2) + "v || Nilai Analog = " + String(analogVolt));
  Serial.println("Sudut Bukaan Servo = "+String(angle, 2));
  Serial.println("-----------------------------------------------------");
  delay(1000);
}
