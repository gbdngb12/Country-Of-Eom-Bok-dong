//#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <mthread.h>

#define LED_PIN 13
//#define BT_RXD 8
//#define BT_TXD 7
//#define GPS_RXD 6
//#define GPS_TXD 5
#define MAGNET 4
#define piezo 12

//SoftwareSerial Serial3(BT_RXD, BT_TXD);

bool lock = false;
bool flag = false;
float diff = 1.5;
typedef struct location {
  float x, y, z;
  float lat, lon;
  bool isMagnetic;
} location;

location loc;
Adafruit_MPU6050 mpu;
TinyGPS gps;

class LoopThread : public Thread {
  public:
    LoopThread(int id);
    static int threadCount;
    static bool isPiezo;
  protected:
    bool loop();
  private:
    int id;
};

static bool LoopThread::LoopThread::isPiezo = false;
static int LoopThread::LoopThread::threadCount = 0;

LoopThread::LoopThread(int id) {
  this->id = id;
  threadCount++;
  if (this->id == 0) { //main Thread
    Serial2.begin(9600);
    Serial3.begin(9600);
    //gps = TinyGPS();
    if (!mpu.begin()) {
      Serial3.println("Failed to find MPU6050");
    } else Serial3.println("MPU6050 Ready");

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ); //센서 필터 설정
    mpu.setMotionDetectionThreshold(1); //모션감지 감도
    mpu.setMotionDetectionDuration(20); //모션감지 간격
    mpu.setMotionInterrupt(true); //모션감지 설정

    pinMode(LED_PIN, OUTPUT);
    pinMode(MAGNET, INPUT);
    //sleep(100);
  }
}



bool LoopThread::loop() {
  //Serial3.print("I am here");
  if (this->id == 0) { //main Thread

    sensors_event_t a, g, temp; //센서값 변수선언
    mpu.getEvent(&a, &g, &temp); //센서값 갱신;

    if (Serial3.available() > 0) {
      int tmp = Serial3.read();
      if (tmp == '0') {
        lock = false;
        digitalWrite(LED_PIN, LOW);
      }
      else if (tmp == '1') {
        lock = true;
        digitalWrite(LED_PIN, HIGH);
      }
    }
    if (Serial2.available() > 0) {
      //Serial3.println("Reading..");
      char c = Serial2.read();
      if (gps.encode(c)) {
        gps.f_get_position(&(loc.lat), &(loc.lon));
        //Serial3.print("Lat/Long: ");
        //Serial3.print(loc.lat, 5);
        //Serial3.print(", ");
        //Serial3.println(loc.lon, 5);
      }
    }
    if (lock == true && flag == false) {
      //Serial3.print("lock!");
      loc.x = g.gyro.x;
      loc.y = g.gyro.y;
      loc.z = g.gyro.z;
      loc.isMagnetic = digitalRead(MAGNET);

      Serial3.println("초기값! :x " + String(loc.x) + "y: " + String(loc.y) + "z: " + String(loc.z) + "자석: " + String(loc.isMagnetic));

      flag = true;
    }
    if (lock == true && flag == true) {
      float x = g.gyro.x;
      float y = g.gyro.y;
      float z = g.gyro.z;

      if (abs(loc.x - x) >= diff) {
        Serial3.println("x: " + String(g.gyro.x) + "이상!");
        isPiezo = true;
        printgps();
      } else if (abs(loc.y - y) >= diff) {
        Serial3.println("y: " + String(g.gyro.y) + "이상!");
        isPiezo = true;
        printgps();
      } else if (abs(loc.z - z) >= diff) {
        Serial3.println("z: " + String(g.gyro.z) + "이상!");
        isPiezo = true;
        printgps();
      }
      if (digitalRead(MAGNET) != loc.isMagnetic) {
        Serial3.println("자석 값 이상!");
        isPiezo = true;
        printgps();
      }
      if ((LoopThread::LoopThread::threadCount < 2) && (isPiezo)) {
        main_thread_list->add_thread(new LoopThread(1)); // create piezo Thread
      }
    }
    if (lock == false) {
      flag = false;
    }
    return true;
  } else if (this->id == 1) { //piezo Thread
    if (kill_flag || lock == false) {
      noTone(piezo);
      threadCount--;
      isPiezo = false;
      return false;
    }
    int toneVal;
    float sinVal;
    for (int x = 0; x < 20; x++) {
      for (int x = 0; x < 360; x++) {
        sinVal = (sin(x * (3.1412 / 180)));
        toneVal = 2000 + (int(sinVal * 1000));
        tone(12, toneVal);

      }
    }
    noTone(12);
    return true;
  }
}


void setup() {
  main_thread_list->add_thread(new LoopThread(0));//Main Thread
}

void printgps() {
  Serial3.print("Lat/Long: ");
  Serial3.print(loc.lat, 5);
  Serial3.print(", ");
  Serial3.println(loc.lon, 5);
}
