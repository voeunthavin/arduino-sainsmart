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
  ControllerBase(void) {
    memset(m_configuration, 0, sizeof(m_configuration));
  }

  virtual ~ControllerBase() {}

  // Path &curve(int drive) { return m_curve[drive]; }

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
    }
  }

  // Should modify this function to take the arguments in order to control the robot manually
  void takeConfigurationValue(float base, float shoulder, float elbow, float roll, float pitch, float wrist, float gripper) {
    m_configuration[0] = clipAngle(0, base);
    m_configuration[1] = clipAngle(1, shoulder);
    m_configuration[2] = clipAngle(2, elbow);
    m_configuration[3] = clipAngle(3, roll);
    m_configuration[4] = clipAngle(4, pitch);
    m_configuration[5] = clipAngle(5, wrist);
    m_configuration[6] = clipAngle(6, gripper);
  }

  float timeRequired(int drive, float angle) {
    return Profile::timeRequired(fabs(angle - target(drive)), MAXJERK);
  }

  float timeRequired() {
    float retval = 0;
    for (int i=0; i<DRIVES; i++) {
      float driveTime = timeRequired(i, m_configuration[i]);
      retval = retval < driveTime ? driveTime : retval;
    }
    return retval;
  }

  void targetAngleUnsafe(int drive, float angle, float time) {
    m_curve[drive].retarget(angle, time);
  }

  void targetPoint() { // The main function that I need to input manually the joints
    float time = timeRequired();
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

  void resetParser(void) {
    memset(m_configuration, 0, sizeof(m_configuration));
  }

  float getRemaining() {
    return m_curve[BASE].timeRemaining();
  }
  float getRequired() {
    return timeRequired();
  }

  void printReportConfig() {
    reportConfiguration(m_curve[0].pos(), m_curve[1].pos(), m_curve[2].pos(),
    m_curve[3].pos(), m_curve[4].pos(), m_curve[5].pos(), m_curve[6].pos());
  }

  // void standbyPoint() {
  //   takeConfigurationValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  //   targetPoint(m_configuration);
  //   printReportConfig();
  //   reportRemaining(getRemaining());
  //   resetParser();
  // }

  // void pickupPoint() {
  //   takeConfigurationValue(0.0, -70.0, 35.0, 50.0, 0.0, -75.0, 90.0);
  //   targetPoint(m_configuration);
  //   reportRemaining(getRemaining());
  //   printReportConfig();
  //   resetParser();
  // }

  // void parseColor(char c) {
  //   // here is the logic to place the object in different location
  //   switch(c) {
  //     case 'w':
  //       takeConfigurationValue(-85.00, -50.00, 10.00, -90.00, -50.00, 90.00, 87.73);
  //       targetPoint(m_configuration);
  //       printReportConfig();
  //       reportRemaining(getRemaining());
  //       resetParser();
  //     break;
  //     case 'r':
  //       takeConfigurationValue(-50.00, -70.00, 10.00, 70.00, -87.99, -20.00, 87.73);
  //       targetPoint(m_configuration);
  //       printReportConfig();
  //       reportRemaining(getRemaining());
  //       resetParser();
  //     break;
  //     case 'g':
  //       takeConfigurationValue(63.00, 23.00, -22.61, 45.00, -60.00, 67.00, 0.00);
  //       targetPoint(m_configuration);
  //       printReportConfig();
  //       reportRemaining(getRemaining());
  //       resetParser();
  //     break;
  //     case 'b':
  //       takeConfigurationValue(83.00, -53.00, -7.00, 90.00, 24.00, 90.00, 24.00);
  //       targetPoint(m_configuration);
  //       printReportConfig();
  //       reportRemaining(getRemaining());
  //       resetParser();
  //     break;
  //     default:
  //       stopDrives();
  //   }
  // }


  virtual int offset(int drive) = 0;
  virtual float resolution(int drive) = 0;
  virtual int lower(int drive) = 0;
  virtual int upper(int drive) = 0;
  virtual void writePWM(int, int) = 0;
  virtual void reportRemaining(float time) = 0;
  virtual void reportRequired(float time) = 0;
  virtual void reportConfiguration(float, float, float, float, float, float, float) = 0;

protected:
  float m_configuration[DRIVES];
  Path m_curve[DRIVES];
};

#endif
