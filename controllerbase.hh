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
  ControllerBase(void): m_index(0) {
    memset(m_configuration, 0, sizeof(m_configuration));
  }

  virtual ~ControllerBase() {}

  Path &curve(int drive) { return m_curve[drive]; }

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

  // Should modify this function to take the arguments in order to control the robot manually
  void takeConfigurationValue(float base, float shoulder, float elbow, float roll, float pitch, float wrist, float gripper) {
    if (m_index < DRIVES) {
      float angle;
      angle = clipAngle(m_index, base);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, shoulder);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, elbow);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, roll);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, pitch);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, wrist);
      m_configuration[m_index++] = angle;
      angle = clipAngle(m_index, gripper);
      m_configuration[m_index++] = angle;
    };
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

  void targetPoint(float point[]) { // The main function that I need to input manually the joints
    float time = timeRequired(point);
    for (int i=0; i<DRIVES; i++)
      targetAngleUnsafe(i, i == ELBOW ? limitArmAngle(ELBOW, point[i]) : point[i], time);
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
    m_index = 0;
  }

  float getRemaining(void) {
    return m_curve[BASE].timeRemaining();
  }

  void parseColor(char c) {
    // The standby joints configuration
    if(getRemaining() == 0.0) {
      takeConfigurationValue(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      targetPoint(m_configuration);
      resetParser();
    }
    // here is the logic to place the object in different location
    switch(c) {
      case 'w':
        if(getRemaining() == 0.0) {
          takeConfigurationValue(53.0, 20.0, 25.0, 50.0, 23.0, 20.0, 19.0);
          targetPoint(m_configuration);
          reportRemaining(m_curve[BASE].timeRemaining());
          reportConfiguration(m_curve[0].pos(), m_curve[1].pos(), m_curve[2].pos(),
          m_curve[3].pos(), m_curve[4].pos(), m_curve[5].pos(), m_curve[6].pos());
          resetParser();
        }
        break;
      case 'r':
        if(getRemaining() == 0.0) {
          takeConfigurationValue(-29.0, -53.0, 28.0, 59.0, 0.0, -20.0, 59.0);
          targetPoint(m_configuration);
          reportRemaining(m_curve[BASE].timeRemaining());
          reportConfiguration(m_curve[0].pos(), m_curve[1].pos(), m_curve[2].pos(),
          m_curve[3].pos(), m_curve[4].pos(), m_curve[5].pos(), m_curve[6].pos());
          resetParser();
        }
        break;
      case 'g':
        if(getRemaining() == 0.0) {
          takeConfigurationValue(63.0, 23.0, -24.0, 45.0, 0.0, 67.0, 0.0);
          targetPoint(m_configuration);
          reportRemaining(m_curve[BASE].timeRemaining());
          reportConfiguration(m_curve[0].pos(), m_curve[1].pos(), m_curve[2].pos(),
          m_curve[3].pos(), m_curve[4].pos(), m_curve[5].pos(), m_curve[6].pos());
          resetParser();
        }
        break;
      case 'b':
        if(getRemaining() == 0.0) {
          takeConfigurationValue(83.0, -53.0, -21.0, 0.0, 24.0, 0.0, 24.0);
          targetPoint(m_configuration);
          reportRemaining(m_curve[BASE].timeRemaining());
          reportConfiguration(m_curve[0].pos(), m_curve[1].pos(), m_curve[2].pos(),
          m_curve[3].pos(), m_curve[4].pos(), m_curve[5].pos(), m_curve[6].pos());
          resetParser();
        }
        break;
      default:
        stopDrives();
    };
  }


  virtual int offset(int drive) = 0;
  virtual float resolution(int drive) = 0;
  virtual int lower(int drive) = 0;
  virtual int upper(int drive) = 0;
  virtual void writePWM(int, int) = 0;
  virtual void reportTime(void) = 0;
  virtual void reportRequired(float time) = 0;
  virtual void reportRemaining(float time) = 0;
  virtual void reportConfiguration(float, float, float, float, float, float, float) = 0;

protected:
  int m_index;
  float m_configuration[DRIVES];
  Path m_curve[DRIVES];
};

#endif
