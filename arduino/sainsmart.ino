#include "../calibration.hh"
#include "../controllerbase.hh"
#include <Adafruit_PWMServoDriver.h>


#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define OUT 8




char color() {
  char c = ' ';

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int red = pulseIn(OUT, LOW);
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int blue = pulseIn(OUT, LOW);
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int green = pulseIn(OUT, LOW);

  Serial.print("RED: ");
  Serial.println(red, DEC);
  Serial.print("GREEN: ");
  Serial.println(green, DEC);
  Serial.print("BLUE: ");
  Serial.println(blue, DEC);

  if (red < blue && red < green && red < 20) {
    if (red <=10 && green <=10 && blue <=10) {
      Serial.println("WHILE");
      c = 'w';
    } else {
      Serial.println(" - (Red Color)");
      c = 'r';
    }
  } else if (blue < red && blue < green) {
    if (red <=10 && green <=10 && blue <= 10){
      Serial.println("WHILE");
      c = 'w';
    } else {
      Serial.println(" - (Blue Color)");
      c = 'b';
    }
  } else if (green < red && green < blue) {
      if (red <= 10 && green <=10 && blue <= 10) {
        Serial.println("WHILE");
        c = 'w';
      } else {
        Serial.println(" - (Green Color)");
        c = 'g';
      }
  } else {
    Serial.println();
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
  void writePWM(int drive, int pwm) {
  	// Convert to Pulse Width from Pulse wide
		int pulse_width = int(float(pwm) / 1000000 * 50 * 4096);
		m_servo.setPWM(drive, 0, pulse_width);
  }
protected:
  Adafruit_PWMServoDriver m_servo = Adafruit_PWMServoDriver();
};


unsigned long t0;

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

  // t0 = millis();
}

void loop() {
  // int dt = millis() - t0;
  controller.parseColor(color());
  // controller.update(dt * 0.001);
  // t0 += dt;
}
