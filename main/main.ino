#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <mthread.h>

#define LED_PIN 13
#define MAGNET 4
#define piezo 12

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

enum status {
  NORMAL = 0,
  URGENCY = 1
};

static bool LoopThread::LoopThread::isPiezo = false;
static int LoopThread::LoopThread::threadCount = 0;

LoopThread::LoopThread(int id) {
  this->id = id;
  threadCount++;
  if (this->id == 0) { //main Thread
    Serial3.begin(9600);
    if (!mpu.begin()) {
      Serial3.println("Failed to find MPU6050");
    } else Serial3.println("MPU6050 Ready");

    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ); //센서 필터 설정
    mpu.setMotionDetectionThreshold(1); //모션감지 감도
    mpu.setMotionDetectionDuration(20); //모션감지 간격
    mpu.setMotionInterrupt(true); //모션감지 설정

    pinMode(LED_PIN, OUTPUT);
    pinMode(MAGNET, INPUT);
  }
}



bool LoopThread::loop() {
  if (this->id == 0) { //main Thread
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
    if (lock == true && flag == false) { //잠금을 거는 상태
      sensors_event_t a, g, temp; //센서값 변수선언
      mpu.getEvent(&a, &g, &temp); //센서값 갱신;

      loc.x = g.gyro.x;
      loc.y = g.gyro.y;
      loc.z = g.gyro.z;
      loc.isMagnetic = digitalRead(MAGNET);
      Serial3.println(NORMAL);
      flag = true;
    }
    if (lock == true && flag == true) { //잠금이 걸려있는 상태
      sensors_event_t a, g, temp; //센서값 변수선언
      mpu.getEvent(&a, &g, &temp); //센서값 갱신;

      float x = g.gyro.x;
      float y = g.gyro.y;
      float z = g.gyro.z;

      if (abs(loc.x - x) >= diff) {
        //Serial3.println("x: " + String(g.gyro.x) + "이상!");
        isPiezo = true;
      } else if (abs(loc.y - y) >= diff) {
        //Serial3.println("y: " + String(g.gyro.y) + "이상!");
        isPiezo = true;
      } else if (abs(loc.z - z) >= diff) {
        //Serial3.println("z: " + String(g.gyro.z) + "이상!");
        isPiezo = true;
      }
      if (digitalRead(MAGNET) != loc.isMagnetic) {
        //Serial3.println("자석 값 이상!");
        isPiezo = true;
      }

      if ((LoopThread::LoopThread::threadCount < 2) && (isPiezo)) {
        //Send to Android URGENCY
        Serial3.println(URGENCY);
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
      Serial3.println(NORMAL);
      return false;
    }
    int toneVal;
    float sinVal;
    for (int x = 0; x < 360; x++) {
      sinVal = (sin(x * (3.1412 / 180)));
      toneVal = 2000 + (int(sinVal * 1000));
      tone(12, toneVal);

    }
    noTone(12);
    return true;
  }
}


void setup() {
  main_thread_list->add_thread(new LoopThread(0));//Main Thread
}
