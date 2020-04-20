#ifndef __PATH_HH
#define __PATH_HH

#include "profile.hh"

class Path {
public:
  Path(void): m_offset(0) {
    m_time[0] = 0;
    m_time[1] = 0;
  }
  
  float pos(void) {
    return m_offset + m_profile[0].value(m_time[0]) + m_profile[1].value(m_time[1]);
  }
  
  float update(float dt) {
    update(dt, m_time[0], m_profile[0]);
    update(dt, m_time[1], m_profile[1]);
    return pos();
  }
  
  void update(float dt, float &time, Profile &profile) {
    time += dt;
    if (time >= profile.getDuration()) {
      m_offset += profile.getDistance();
      profile.reset();
    };
  }
  void stop(float pos) {
    m_offset = pos;
    m_profile[0].reset();
    m_profile[1].reset();
  }
  bool ready(void) {
    return m_profile[0].ifEmpty() || m_profile[1].ifEmpty();
  }
  void retarget(float target, float duration) {
    retarget(target, duration, m_time[0], m_profile[0]) || retarget(target, duration, m_time[1], m_profile[1]);
  }
  float target(void) {
    return m_offset + m_profile[0].getDistance() + m_profile[1].getDistance();
  }
  bool retarget(float value, float duration, float &time, Profile &profile) {
    if (!profile.ifEmpty())
      return false;
    else {
      time = 0;
      profile.reset(value - target(), duration);
      return true;
    };
  }
  float timeRemaining(float time, Profile &profile) {
    return profile.ifEmpty() ? 0 : profile.getDuration() - time;
  }
  float timeRemaining(void) {
    float a = timeRemaining(m_time[0], m_profile[0]);
    float b = timeRemaining(m_time[1], m_profile[1]);
    return a >= b ? a : b;
  }
  
protected:
  Profile m_profile[2];
  float m_time[2];
  float m_offset;
};

#endif
