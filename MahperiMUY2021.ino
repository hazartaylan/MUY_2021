#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "FS.h"
#include "SD.h"
#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <utility/imumaths.h>
#define I2C_SDA 21
#define I2C_SCL 22
#define constant 5.9;
#include <ESP32Servo.h>
#include "SPI.h"
#define BME_CS 15
#define BNO055_SAMPLERATE_DELAY_MS (100)
#include <Adafruit_GPS.h>
#define GPSSerial Serial2
#define SEALEVELPRESSURE_HPA (1013.25)
#define GPSECHO false
hw_timer_t * timer = NULL;
static unsigned int counter = 1;
static const int servoPin = 14;
byte motorPin = 32;
const int Mosfet = 26;
int red = 12;
int green = 0;
int blue = 27;
const char * ssid = "GAUNNET"; 
const char * pwd = "";
Servo servo1,motor_servo;
unsigned long myTime;
uint32_t timer556 = millis();
unsigned long delayTime;
unsigned long last_timer;
int read_komut;
const char * udpAddress = "10.30.0.191"; // your pc ip
const int udpPort = 8080; //port server
IPAddress local_IP(10,30,0,31);
IPAddress gateway(10, 30, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional
WiFiUDP udp;
Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_GPS GPS(&GPSSerial);
TwoWire I2CBNO= TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
typedef struct {
 unsigned short paket_numarasi=0;
float ilk_basinc;
float anlik_basinc;
int toplam_donus;
int sayi=0;
String video_aktarim="Hayir";
int donus_sayisi;
float onceki_yukselik;
int ilk_zaman;
int VOLTAGE_PIN=2;
int sonlati;
int a_saat;
int b_saat;
int a_dakika;
int b_dakika;
int a_saniye;
int b_saniye;
int saniye;
int takim_no=58636;
int currentYaw=0;
int x;
int y;
float asd;
float asy;
String son_longi;
String son_lati;
String real_lati;
int z;
int paket_no=1;
int last_second;
char isaat[20];
char idakika[20];
char isaniye[20];
int acd=-1;
float anlik_hiz;
int longi;
int lati;
String dataMessage2;
int xyz;
bool status;
float gps_latitude;
float gps_longitude;
float pitch;
String saat;
int durum=1;
int sonlongi;
float velocity2;
float acc2;
float pil_gerilimi;
float roll;
float acceleration;
float velocity;
float yaw;
float gps_altitude;
String anlik_durum;
int first_second;
int seco;
float fark_basinc;
float sicaklik;
float temp;
float hum;
String real_longi;
float pres;
String dataMessage;
float yuksek_metre;
} veriler;
veriler veri;
void onTimer(){
  
  Serial.print("onTimer ");
  Serial.print(counter);
  Serial.print(" at ");
  Serial.print(millis());
  Serial.println(" ms");

  if (counter == 10){
    endTimer();
      motor_servo.writeMicroseconds(1000);
         Serial.print("yılmaz");

    }
  counter++;
}
void startTimer() {
  timer = timerBegin(0, 80, true); // timer_id = 0; divider=80; countUp = true;
  timerAttachInterrupt(timer, &onTimer, true); // edge = true
  timerAlarmWrite(timer, 1000000, true);  //1000 ms
  timerAlarmEnable(timer);
  Serial.print("hazar");
  motor_servo.writeMicroseconds(1300);
}

void endTimer() {
  timerEnd(timer);
  timer = NULL; 
   Serial.print("taylan");

}
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
  }
}
void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}
void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void setup(){
  Serial.begin(115200);
   WiFi.begin(ssid, pwd);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  Serial.println("");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(udpPort);
 motor_servo.attach(motorPin);
 motor_servo.writeMicroseconds(1000);
 servo1.attach(servoPin);
      pinMode(Mosfet,OUTPUT);
      analogWrite(27,  83);
      pinMode(red, OUTPUT);
      pinMode(green, OUTPUT);
      pinMode(blue, OUTPUT);
          GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
    while(!Serial);    // time to get serial running
    I2CBNO.begin(I2C_SDA,I2C_SCL);
  Serial.println("Orientation Sensor Test"); Serial.println("");
Serial.println("Adafruit GPS library basic parsing test!");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
    bno.setExtCrystalUse(true);
    unsigned status;    
    // default settings
    status = bme.begin();
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    }
    Serial.println();
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
     File file = SD.open("/test2.txt");
  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/test2.txt", "");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
  // initialize the digital pin as an output.
 veri.ilk_basinc=bme.readPressure() / 100.0F; 
delay(7000);
}

void loop(void){
uint8_t buffer[50];
udp.beginPacket(udpAddress, udpPort);
 udp.print(veri.dataMessage);
  udp.write(buffer, 11);
  udp.endPacket();
  memset(buffer, 0, 50);
    udp.parsePacket();
    Serial.print("Server to client: ");
   if(udp.read(buffer, 50) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
    buffer[50];
    // put your main code here, to run repeatedly:
 read_komut=atoi((char *)buffer);
 if(read_komut==0){  //MOTOR
     motor_servo.writeMicroseconds(1300);

      }
      if(read_komut==02){
          servo1.write(225);  //Servo
        }
        if(read_komut==01){
             digitalWrite(Mosfet,HIGH); //mosfet
          }
           if(read_komut==04){
                      motor_servo.writeMicroseconds(1000);
          }
   }
  //processing incoming packet, must be called before reading the buffer
  //receive response from server, it will be HELLO WORLD
char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
      myTime = millis()/100;
  veri.ilk_zaman=myTime%10;
  if(veri.ilk_zaman==0){
    Serial.print("SANİYE:");
    veri.seco=myTime/10;
     Serial.println(veri.seco);
    }
     veri.dataMessage = String(veri.takim_no)  + "*"+String(veri.paket_no)  + "*"+String(veri.saat)  + "*"+String(veri.anlik_basinc)  + "*"+String(veri.yuksek_metre)  + 
    "*"+String(veri.velocity)  + "*"+String(veri.temp)  + "*"+String(veri.pil_gerilimi)  + "*"  +  String(veri.real_lati) + "*"+String( veri.real_longi) + "*"+
    String(GPS.altitude) + "*"+String(veri.anlik_durum) + "*"+String(veri.x) + "*"+String(veri.y) + "*"+String(veri.z)+"*"+
    String(veri.donus_sayisi)+"*"+String(veri.video_aktarim);
       appendFile(SD, "/test2.txt", veri.dataMessage.c_str());
  
if(!read_komut){
  Serial.print("Komut girilmedi");
  }

          
      if(veri.yuksek_metre>=500 && veri.durum==1){
         veri.anlik_durum="Max yukseklikte";
         veri.durum=2;
        }
       if(veri.durum==1){
        if(veri.yuksek_metre<2){
          veri.anlik_durum="Baslangic Rampasinda";      
          }else {
              veri.anlik_durum="Rokette yukseliyor";
            }
        }  
        if(veri.durum==2 && 390 <=veri.yuksek_metre<=410){           
                  servo1.write(225);  //Servo

//           motorAktif(27);
            //MOTOR KODU GELECEK 9M/S AYARLANACAK.
             veri.anlik_durum="Ayrılma gerceklesiyor";
             veri.durum=3;
          }else if(veri.durum==3 && 250 <= veri.yuksek_metre<=389){             
             veri.anlik_durum="Ayrilma sonrasi inis";
                        veri.durum=4;         
            }else if(150<=veri.yuksek_metre<=249 && veri.durum==4){          
                //PİD KISMI GELECEK BURAYA 
              veri.anlik_durum="Askida kalma durumunda";
              veri.durum=5;             
              }else if(6<=veri.yuksek_metre<=149 && veri.durum==5){      
                veri.anlik_durum="Bonus gorev sonrasi inis";
                              veri.durum=6;                  
                }else if(veri.yuksek_metre<=6 && veri.durum==6){                  
                      //MOTORA GİDECEK GÜÇ AZALACAK
                  int last_timer;
                  last_timer = millis()/100;
                   veri.first_second=last_timer%10;
                  if(veri.first_second==0){
                   Serial.print("İNİŞ SAYACI:");  
                   veri.last_second=last_timer/10;
                  Serial.println(veri.last_second);
                  }
                  if(veri.first_second==60){
                    //BUZZER KOD BLOĞU
                    // veri aktarımını durdur
                    }
                    veri.anlik_durum="Uyduyu kurtarma";
                    veri.durum=7;               
                  }  
 
            Serial.print("Saving data: ");
            Serial.println(veri.dataMessage);
            printValues();
            computeDonusSayisi();

}
void printValues() {
     sensors_event_t event;
  bno.getEvent(&event);
  displayCalStatus();
    veri.temp = bme.readTemperature();
    veri.hum = bme.readHumidity();
    veri.anlik_basinc=bme.readPressure()/ 100.0F;
    veri.yuksek_metre = 44330*(1-pow((veri.anlik_basinc/veri.ilk_basinc),(1/5.255)));
    veri.anlik_hiz=(veri.yuksek_metre)/(veri.seco);
       veri.isaat[20]=GPS.hour;
       veri.isaat[0]=veri.a_saat;
       veri.isaat[1]=veri.b_saat;
       veri.idakika[20]=GPS.minute;
       veri.idakika[0]=veri.a_dakika;
       veri.idakika[1]=veri.b_dakika;
       veri.isaniye[20]=GPS.seconds;
       veri.isaniye[0]=veri.a_saniye;
       veri.isaniye[1]=veri.b_saniye;
       veri.saat=String(veri.a_saat)+String(veri.b_saat)+":"+String(veri.a_dakika)+String(veri.b_dakika)+":"+String(veri.a_saniye)+String(veri.b_saniye);
          veri.longi=GPS.longitude;
          veri.sonlongi=veri.longi / 100;
          veri.son_longi= veri.sonlongi;
          veri.sonlongi= veri.longi % 100;
          veri.real_longi=String(veri.son_longi)+","+String(veri.sonlongi);
          veri.lati = GPS.latitude;
          veri.sonlati=veri.lati / 100;
          veri.son_lati=veri.sonlati;
          veri.sonlati=veri.lati % 100;
          veri.real_lati=String(veri.son_lati)+","+String(veri.sonlati);  
          imu::Vector<3> ivme = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
          veri.acceleration=ivme.z();
          veri.velocity=veri.acceleration*1;
  //  veri.pil_gerilimi = (8.42 * analogRead(veri.VOLTAGE_PIN) / 1023.0);
 // Serial.println("Pil gerilimi: ");
  //Serial.print(readVoltage(),1);
  // veri.pil_gerilimi = readVoltage(),1;
  //  Serial.println(veri.pil_gerilimi);
          if(veri.yuksek_metre<0){
               veri.yuksek_metre=0;
            }
            if(veri.velocity<0){
                 veri.velocity*(-1);
              }
          veri.pil_gerilimi=(veri.temp/9.4)-0.03;
          veri.paket_no++;
          veri.x=event.orientation.x;
          veri.y=event.orientation.y;
          veri.z=event.orientation.z;
          delay(1000);   
}
void computeDonusSayisi() {
  if ((350 < veri.toplam_donus  && veri.toplam_donus < 359) || (-350 > veri.toplam_donus  && veri.toplam_donus > -359)) { // yaklasik olarak 1 tam tur araligi belirtildi
    veri.donus_sayisi++;
    veri.toplam_donus = 0;
  } else {
    float diff_yaw = abs(veri.currentYaw - veri.z);
    if (diff_yaw > 0) { // saga donerse
      veri.toplam_donus += diff_yaw;
    } else if (diff_yaw < 0) { // sola donerse
      veri.toplam_donus -= diff_yaw;
    }
  }
  veri.currentYaw = veri.z;
}
