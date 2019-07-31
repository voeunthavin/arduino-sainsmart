#ifndef __CONTROLLER_HH
#define __CONTROLLER_HH

#include "calibration.hh"
#include "path.hh"

const int BASE     = 0;
const int SHOULDER = 1;
const int ELBOW    = 2;
const int ROLL     = 3;
const int PITCH    = 4;
const int WRIST    = 5;
const int GRIPPER  = 6;
const int DRIVES   = 7;

const float ELBOW_RANGE = 60;

class ControllerBase {
public:
  ControllerBase(void): m_number(0), m_fraction(0), m_sign(0), m_index(0) {
    memset(m_configuration, 0, sizeof(m_configuration));
  }

  virtual ~ControllerBase() {}

  Path &curve(int drive) { return m_curve[drive]; }

  int drive(char c) {
    switch (tolower(c)) {
    case 's':
      return SHOULDER;
    case 'e':
      return ELBOW;
    case 'r':
      return ROLL;
    case 'p':
      return PITCH;
    case 'w':
      return WRIST;
    case 'g':
      return GRIPPER;
    default:
      return BASE;
    };
  }

  float target(int drive) {
    return m_curve[drive].target();
  }

  float limit(float value, float lower, float upper) { // make sure the pwm is within the boundaries
    return value < lower ? lower : value > upper ? upper : value;
  }

  float angleToPWM(int drive, float angle) {
    return offset(drive) + angle * resolution(drive);
  }

  float pwmToAngle(int drive, float pwm) {
    return (pwm - offset(drive)) / resolution(drive);
  }

  float clipPWM(int drive, float value) { // make sure drive takes the pwm that's within the ranges
    return limit(value, lower(drive), upper(drive));
  }

  float clipAngle(int drive, float value) { // just make sure the angle is valid for the joint range
    return pwmToAngle(drive, clipPWM(drive, angleToPWM(drive, value)));
  }

  float limitJoint(float value, float other) {
    return limit(value, -ELBOW_RANGE - other, ELBOW_RANGE - other);
  }

  float limitArmAngle(int drive, float value) {
    switch (drive) {
    case ELBOW:
      return limitJoint(value, target(SHOULDER));
    case SHOULDER:
      return limitJoint(value, target(ELBOW));
    default:
      return value;
    };
  }

  void takeConfigurationValue(float num[]) { // Should modify this function to take the arguments in order to control the robot manually
    for(m_index; m_index < DRIVES; m_index++) {
      float angle = clipAngle(m_index, num[m_index]);
      m_configuration[m_index] = angle;
    }
    resetNumber();
  }

  float timeRequired(int drive, float angle) {
    return Profile::timeRequired(fabs(angle - target(drive)), MAXJERK);
  }

  float timeRequired(float point[]) {
    float retval = 0;
    for (int i=0; i<DRIVES; i++) {
      float driveTime = timeRequired(i, point[i]);
      retval = retval < driveTime ? driveTime : retval;
    };
    return retval;
  }

  void targetAngleUnsafe(int drive, float angle, float time) {
    m_curve[drive].retarget(angle, time);
  }

  void targetPWM(int drive, float pwm) {
    float angle = limitArmAngle(drive, pwmToAngle(drive, clipPWM(drive, pwm)));
    targetAngleUnsafe(drive, angle, timeRequired(drive, angle));
  }

  void targetAngle(int drive, float value) {
    float angle = limitArmAngle(drive, clipAngle(drive, value));
    targetAngleUnsafe(drive, angle, timeRequired(drive, angle));
  }

  void targetPoint(void) { // The main function that I need to input manually the joints
    float time = timeRequired(m_configuration);
    for (int i=0; i<DRIVES; i++)
      targetAngleUnsafe(i, i == ELBOW ? limitArmAngle(ELBOW, m_configuration[i]) : m_configuration[i], time);
  }

  void update(float dt) {
    for (int drive=0; drive<DRIVES; drive++)
      writePWM(drive, round(angleToPWM(drive, m_curve[drive].update(dt))));
  }

  void stopDrives(void) {
    for (int drive=0; drive<DRIVES; drive++)
      m_curve[drive].stop(m_curve[drive].pos());
  }

  float number(void) {
    float fraction = (m_fraction == 0) ? 1 : m_fraction;
    return m_number * fraction * m_sign;
  }

  void resetNumber(void) {
    m_number = 0;
    m_fraction = 0;
    m_sign = 0;
  }

  virtual void writePWM(int, int) = 0;

protected:
  float m_number;
  float m_fraction;
  char m_sign;
  int m_index;
  float m_configuration[DRIVES];
  Path m_curve[DRIVES];
};

#endif
