#include "../calibration.hh"
#include "../controllerbase.hh"
#include <Adafruit_PWMServoDriver.h>


#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define OUT 8


char getColor() {
  char c = 'x';

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int red = pulseIn(OUT, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int blue = pulseIn(OUT, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int green = pulseIn(OUT, LOW);

  if (red < blue && red < green && red < 20) {
    if (red <=10 && green <=10 && blue <=10) {
      // Serial.println("WHILE");
      c = 'w';
    } else {
      c = 'r';
      // Serial.println(" - (Red Color)");
    }
  } else if (blue < red && blue < green) {
    if (red <=10 && green <=10 && blue <= 10){
      // Serial.println("WHILE");
      c = 'w';
    } else {
      // Serial.println(" - (Blue Color)");
      c = 'b';
    }
  } else if (green < red && green < blue) {
      if (red <= 10 && green <=10 && blue <= 10) {
        // Serial.println("WHILE");
        c = 'w';
      } else {
        // Serial.println(" - (Green Color)");
        c = 'g';
      }
  } else {
    c = 'x';
  }

  return c;
}


class Controller: public ControllerBase {
public:
  Controller(void) {}
	void setup(void) {
		m_servo.begin();
		m_servo.setPWMFreq(50);
	}
  int offset(int drive) { return OFFSET[drive]; }
  float resolution(int drive) { return RESOLUTION[drive]; }
  int lower(int drive) { return MIN[drive]; }
  int upper(int drive) { return MAX[drive]; }
  void reportInteger(int value) {
    Serial.print(value);
    Serial.write("\r\n");
  }
  void reportFloat(float value) {
    Serial.print(value);
    Serial.write("\r\n");
  }
  void reportRemaining(float time) {
    reportFloat(time);
  }
  void reportRequired(float time) {
    reportFloat(time);
  }
  void reportConfiguration(float base, float shoulder, float elbow, float roll, float pitch, float wrist, float gripper) {
    Serial.print(base);
    Serial.write(" ");
    Serial.print(shoulder);
    Serial.write(" ");
    Serial.print(elbow);
    Serial.write(" ");
    Serial.print(roll);
    Serial.write(" ");
    Serial.print(pitch);
    Serial.write(" ");
    Serial.print(wrist);
    Serial.write(" ");
    Serial.print(gripper);
    Serial.write("\r\n");
  }
  void writePWM(int drive, int pwm) {
  	// Convert to Pulse Width from Pulse wide
		int pulse_width = int(float(pwm) / 1000000 * 50 * 4096);
		m_servo.setPWM(drive, 0, pulse_width);
  }
protected:
  Adafruit_PWMServoDriver m_servo = Adafruit_PWMServoDriver();
};


unsigned long t0;
char color;
bool flag[6];

Controller controller;


void setup() {
  controller.setup();
  Serial.begin(115200);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  t0 = 0;
  flag[0] = true;
  flag[1] = false;
  flag[2] = false;
  flag[3] = false;
  flag[4] = false;
  flag[5] = false;
  flag[6] = false;
}

void loop() {
  unsigned long current = millis() - t0;
  if(current * 0.001 > 0.0 && controller.getRemaining() <= 0.01) {
    if(flag[0]) {
      controller.takeConfigurationValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      controller.targetPoint();
      controller.resetParser();
      color = getColor();
      flag[0] = false;
      flag[1] = true;
    }
  }
  if(current * 0.001 > 2.0) {
    if(color != 'x') {
      if(flag[1]) {
        controller.takeConfigurationValue(5.00, -75.00, 15.00, -5.00, -50.00, 0.00, 0.0); // pick
        controller.targetPoint();
        controller.resetParser();
        flag[1] = false;
        flag[3] = true;
      }
      if(flag[3] && controller.getRemaining() <= 0.01) {
        controller.takeConfigurationValue(5.00, -75.00, 15.00, -5.00, -50.00, 0.00, 40.0); // grap!!
        controller.targetPoint();
        controller.resetParser();
        flag[3] = false;
        flag[4] = true;
      }
      if(flag[4] && controller.getRemaining() <= 0.01) {
        flag[4] = false;
        flag[2] = true; // go to place position
      }
    } else {
      t0 = millis();
      flag[0] = true;
      flag[2] = false;
    }
  }
  if(current * 0.001 > 5.5) {
    if(flag[2]) {
      switch(color) {
        case 'w':
          controller.takeConfigurationValue(-85.00, -50.00, 10.00, -90.00, -50.00, 90.00, 40.0);
          break;
        case 'r':
          controller.takeConfigurationValue(-50.00, -70.00, 10.00, 70.00, 20.0, -60.00, 40.0);
          break;
        case 'g':
          controller.takeConfigurationValue(63.00, 23.00, -22.61, 45.00, -60.00, 67.00, 40.0);
          break;
        case 'b':
          controller.takeConfigurationValue(83.00, -53.00, -7.00, -45.00, 24.00, 0.00, 40.0);
          break;
      }
      controller.targetPoint();
      controller.resetParser();
      flag[2] = false;
      flag[5] = true; // drop it!!
    }
    if(flag[5] && controller.getRemaining() <= 0.01) {
      switch(color){
        case 'w':
          controller.takeConfigurationValue(-85.00, -50.00, 10.00, -90.00, -50.00, 90.00, 0.0);
          break;
        case 'r':
          controller.takeConfigurationValue(-50.00, -70.00, 10.00, 70.00, 20.0, -60.00, 0.0);
          break;
        case 'g':
          controller.takeConfigurationValue(63.00, 23.00, -22.61, 45.00, -60.00, 67.00, 0.0);
          break;
        case 'b':
          controller.takeConfigurationValue(83.00, -53.00, -7.00, -45.00, 24.00, 0.0, 0.0);
          break;
      }
      controller.targetPoint();
      controller.resetParser();
      flag[5] = false;
      flag[6] = true; // reset and scan color
    }

    if(flag[6] && controller.getRemaining() <= 0.01) {
      t0 = millis();
      flag[0] = true;
      flag[6] = false;
    }
  }

  Serial.println(controller.getRemaining());
  controller.printReportConfig();
  Serial.println(current * 0.001);
  Serial.println(color);
  controller.update(0.01);
}
