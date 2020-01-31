#include "../calibration.hh"
#include "../controllerbase.hh"
#include <Adafruit_PWMServoDriver.h>

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
  void reportConfiguration(float base, float shoulder, float elbow, float roll, float pitch, float wrist) {
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
bool flag[8];

Controller controller;


void setup() {
  controller.setup();
  Serial.begin(115200);
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
      controller.takeConfigurationValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      controller.targetPoint();
      controller.resetParser();
      // if getRemaining == 0.0 ??
      flag[0] = false;
      flag[1] = true;
    }
  }
  if(current * 0.001 > 2.0) {
    if(flag[1]) {
      controller.takeConfigurationValue(5.00, -75.00, 15.00, -5.00, -55.00, 0.00); // pick
      controller.targetPoint();
      controller.resetParser();
      flag[1] = false;
      flag[3] = true;
    }
    if(flag[3] && controller.getRemaining() <= 0.01) {
      controller.takeConfigurationValue(5.00, -75.00, 15.00, -5.00, -55.00, 0.00); // grap!!
      controller.targetPoint();
      controller.resetParser();
      flag[3] = false;
      flag[4] = true; // go back to standby point
    }
    if(flag[4] && controller.getRemaining() <= 0.01) {
      flag[4] = false;
      flag[2] = true; // go to place position
    }
  }
  if(current * 0.001 > 5.5) {
    if(flag[2]) {
      controller.takeConfigurationValue(-85.00, -50.00, 10.00, -90.00, -50.00, 90.00);
      controller.targetPoint();
      controller.resetParser();
      flag[2] = false;
      flag[5] = true; // drop it!!
    }
    if(flag[5] && controller.getRemaining() <= 0.01) {
      controller.takeConfigurationValue(-50.00, -70.00, 10.00, 70.00, 20.0, -60.00);
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
  controller.update(0.01);
}
