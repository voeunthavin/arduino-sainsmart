#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "controller.hh"
#include "curve.hh"

class StationaryTest: public ::testing::Test {
public:
  StationaryTest(void) { m_curve.setBound(10); }
protected:
  Curve m_curve;
};

TEST_F(StationaryTest, StartWithGivenValue) {
  EXPECT_EQ(0, m_curve.pos());
  EXPECT_EQ(0, m_curve.target());
}

TEST_F(StationaryTest, StayConstantOverTime) {
  m_curve.update(5);
  EXPECT_EQ(0, m_curve.pos());
}

class MovingForwardTest: public ::testing::Test {
public:
  MovingForwardTest(void)
  {
    m_curve.setBound(10);
    m_curve.setPos(100);
    m_curve.retarget(140);
  }
protected:
  Curve m_curve;
};

TEST_F(MovingForwardTest, RememberTarget) {
  EXPECT_EQ(140, m_curve.target());
}

TEST_F(MovingForwardTest, AdvanceWithTime) {
  m_curve.update(1);
  EXPECT_EQ(105, m_curve.pos());
}

TEST_F(MovingForwardTest, AccelerateOverTime) {
  m_curve.update(1);
  m_curve.update(1);
  EXPECT_EQ(120, m_curve.pos());
}

TEST_F(MovingForwardTest, UseSpecifiedAcceleration) {
  Curve curve;
  curve.setBound(20);
  curve.retarget(200);
  curve.update(1);
  EXPECT_EQ(10, curve.pos());
}

TEST_F(MovingForwardTest, UpdateReturnsPosition) {
  Curve curve;
  curve.setBound(20);
  curve.retarget(200);
  EXPECT_EQ(10, curve.update(1));
  EXPECT_EQ(40, curve.update(1));
}

TEST_F(MovingForwardTest, DetermineTimeOfReversal) {
  EXPECT_EQ(2, m_curve.reverseTime());
  m_curve.update(1);
  EXPECT_EQ(1, m_curve.reverseTime());
  m_curve.update(1);
  EXPECT_EQ(0, m_curve.reverseTime());
}

TEST_F(MovingForwardTest, DecelerateBeforeTarget) {
  m_curve.update(3);
  EXPECT_EQ(135, m_curve.pos());
}

TEST_F(MovingForwardTest, StopAtTarget) {
  m_curve.update(4);
  EXPECT_EQ(140, m_curve.pos());
  m_curve.update(1);
  EXPECT_EQ(140, m_curve.pos());
}

TEST_F(MovingForwardTest, AccomodateReducedTarget) {
  m_curve.update(2);
  m_curve.retarget(120);
  m_curve.update(0);
  EXPECT_EQ(120, m_curve.pos());
}

TEST_F(MovingForwardTest, RampDownForReducedTarget) {
  m_curve.update(2);
  m_curve.retarget(120);
  m_curve.update(2);
  EXPECT_EQ(140, m_curve.pos());
}

TEST_F(MovingForwardTest, ContinueIfIdentical) {
  m_curve.update(2);
  m_curve.retarget(140);
  m_curve.update(2);
  EXPECT_EQ(140, m_curve.pos());
}

TEST_F(MovingForwardTest, AbortMotion) {
  m_curve.update(2);
  m_curve.stop();
  m_curve.update(2);
  EXPECT_EQ(120, m_curve.pos());
}

class MovingBackwardTest: public ::testing::Test {
public:
  MovingBackwardTest(void)
  {
    m_curve.setBound(10);
    m_curve.setPos(100);
    m_curve.retarget(60);
  }
protected:
  Curve m_curve;
};

TEST_F(MovingBackwardTest, DetermineTimeOfReversal) {
  EXPECT_EQ(2, m_curve.reverseTime());
  m_curve.update(1);
  EXPECT_EQ(1, m_curve.reverseTime());
  m_curve.update(1);
  EXPECT_EQ(0, m_curve.reverseTime());
}

TEST_F(MovingBackwardTest, AdvanceWithTime) {
  m_curve.update(1);
  EXPECT_EQ(95, m_curve.pos());
}

class MockController: public ControllerBase
{
public:
  int offset(int drive) { return 1500; }
  float resolution(int drive) { return 12.0; }
  int lower(int drive) { return 544; }
  int upper(int drive) { return 2400; }
  MOCK_METHOD0(reportTime, void());
  MOCK_METHOD1(reportAngle, void(float));
  MOCK_METHOD1(reportPWM, void(float));
  MOCK_METHOD0(stopDrives, void());
};

class ControllerTest: public ::testing::Test {
public:
  ControllerTest(void) {
    m_controller.curve(BASE).setPos(45);
    m_controller.curve(SHOULDER).setPos(-10);
    m_controller.curve(ELBOW).setPos(20);
    m_controller.curve(GRIPPER).setPos(30);
  }
protected:
  MockController m_controller;
};

TEST_F(ControllerTest, CheckTime) {
  EXPECT_CALL(m_controller, reportTime());
  m_controller.parseChar('t');
}

TEST_F(ControllerTest, ReportBase) {
  EXPECT_CALL(m_controller, reportAngle(45));
  m_controller.parseChar('b');
}

TEST_F(ControllerTest, ReportBasePWM) {
  EXPECT_CALL(m_controller, reportPWM(2040.0));
  m_controller.parseChar('B');
}

TEST_F(ControllerTest, ClipAcceptsValues) {
  EXPECT_EQ(1500, m_controller.clip(BASE, 1500));
}

TEST_F(ControllerTest, ClipLowerBound) {
  EXPECT_EQ(544, m_controller.clip(BASE, 500));
}

TEST_F(ControllerTest, ClipUpperBound) {
  EXPECT_EQ(2400, m_controller.clip(BASE, 2500));
}

TEST_F(ControllerTest, ConvertZeroAngleToPWM) {
  EXPECT_EQ(1500, m_controller.angleToPWM(SHOULDER, 0));
}

TEST_F(ControllerTest, ConvertAngleToPWM) {
  EXPECT_EQ(1740, m_controller.angleToPWM(SHOULDER, 20));
}

TEST_F(ControllerTest, ConvertCenterPWMToAngle) {
  EXPECT_EQ(0, m_controller.pwmToAngle(BASE, 1500));
}

TEST_F(ControllerTest, ConvertPWMToAngle) {
  EXPECT_EQ(20, m_controller.pwmToAngle(BASE, 1740));
}

TEST_F(ControllerTest, TargetBaseAngle) {
  m_controller.parseChar('1');
  m_controller.parseChar('2');
  m_controller.parseChar('b');
  EXPECT_EQ(12, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, TargetBasePWM) {
  m_controller.parseChar('1');
  m_controller.parseChar('3');
  m_controller.parseChar('2');
  m_controller.parseChar('0');
  m_controller.parseChar('B');
  EXPECT_EQ(-15.0, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, RetargetBaseFloat) {
  m_controller.parseChar('1');
  m_controller.parseChar('2');
  m_controller.parseChar('.');
  m_controller.parseChar('5');
  m_controller.parseChar('b');
  EXPECT_EQ(12.5, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, TargetBasePWMFloat) {
  m_controller.parseChar('1');
  m_controller.parseChar('5');
  m_controller.parseChar('0');
  m_controller.parseChar('1');
  m_controller.parseChar('.');
  m_controller.parseChar('5');
  m_controller.parseChar('B');
  EXPECT_EQ(0.125, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, TargetBasePWMLowerLimit) {
  m_controller.parseChar('0');
  m_controller.parseChar('B');
  EXPECT_LT(-80, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, TargetShoulderPWMLimitAgainstElbow) {
  m_controller.parseChar('2');
  m_controller.parseChar('4');
  m_controller.parseChar('0');
  m_controller.parseChar('0');
  m_controller.parseChar('S');
  EXPECT_GE(35, m_controller.curve(SHOULDER).target());
}

TEST_F(ControllerTest, RetargetNegativeNumber) {
  m_controller.parseChar('-');
  m_controller.parseChar('1');
  m_controller.parseChar('.');
  m_controller.parseChar('5');
  m_controller.parseChar('b');
  EXPECT_EQ(-1.5, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, DoubleMinus) {
  m_controller.parseChar('-');
  m_controller.parseChar('-');
  m_controller.parseChar('1');
  m_controller.parseChar('b');
  EXPECT_EQ(1, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, RetargetZero) {
  m_controller.parseChar('0');
  m_controller.parseChar('b');
  EXPECT_EQ(0, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, TargetTwice) {
  m_controller.parseChar('3');
  m_controller.parseChar('b');
  m_controller.parseChar('3');
  m_controller.parseChar('b');
  EXPECT_EQ(3, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, IgnoreInvalidFloat) {
  m_controller.parseChar('3');
  m_controller.parseChar('.');
  m_controller.parseChar('4');
  m_controller.parseChar('.');
  m_controller.parseChar('5');
  m_controller.parseChar('b');
  EXPECT_EQ(5, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, AbortRetargetBase) {
  EXPECT_CALL(m_controller, stopDrives());
  EXPECT_CALL(m_controller, reportAngle(45));
  m_controller.parseChar('5');
  m_controller.parseChar('x');
  m_controller.parseChar('b');
}

TEST_F(ControllerTest, CorrectTarget) {
  EXPECT_CALL(m_controller, stopDrives());
  m_controller.parseChar('5');
  m_controller.parseChar('x');
  m_controller.parseChar('3');
  m_controller.parseChar('b');
  EXPECT_EQ(3, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, RetargetBaseOnce) {
  EXPECT_CALL(m_controller, reportAngle(45));
  m_controller.parseChar('3');
  m_controller.parseChar('0');
  m_controller.parseChar('b');
  m_controller.parseChar('b');
  EXPECT_EQ(30, m_controller.curve(BASE).target());
}

TEST_F(ControllerTest, ReportElbow) {
  EXPECT_CALL(m_controller, reportAngle(20));
  m_controller.parseChar('e');
}

TEST_F(ControllerTest, ReportElbowPWM) {
  EXPECT_CALL(m_controller, reportPWM(1740.0));
  m_controller.parseChar('E');
}

TEST_F(ControllerTest, RetargetElbow) {
  m_controller.parseChar('1');
  m_controller.parseChar('0');
  m_controller.parseChar('e');
  EXPECT_EQ(10, m_controller.curve(ELBOW).target());
}

TEST_F(ControllerTest, ReportShoulder) {
  EXPECT_CALL(m_controller, reportAngle(-10));
  m_controller.parseChar('s');
}

TEST_F(ControllerTest, ReportShoulderPWM) {
  EXPECT_CALL(m_controller, reportPWM(1380.0));
  m_controller.parseChar('S');
}

TEST_F(ControllerTest, ReportGripper) {
  EXPECT_CALL(m_controller, reportAngle(30));
  m_controller.parseChar('g');
}

TEST_F(ControllerTest, ReportGripperPWM) {
  EXPECT_CALL(m_controller, reportPWM(1860.0));
  m_controller.parseChar('G');
}

TEST_F(ControllerTest, AbortPathForOtherKey) {
  EXPECT_CALL(m_controller, stopDrives());
  m_controller.parseChar('x');
}

TEST_F(ControllerTest, UseBase) {
  EXPECT_EQ(-90, m_controller.limitArm(BASE, -90));
  EXPECT_EQ( 90, m_controller.limitArm(BASE,  90));
}

TEST_F(ControllerTest, RestrictElbow) {
  m_controller.curve(SHOULDER).setPos(0);
  EXPECT_EQ( 45, m_controller.limitArm(ELBOW,  70));
  EXPECT_EQ(-45, m_controller.limitArm(ELBOW, -70));
}

TEST_F(ControllerTest, RestrictElbowRelativeToShoulder) {

  EXPECT_EQ( 55, m_controller.limitArm(ELBOW, 70));
  EXPECT_EQ(-35, m_controller.limitArm(ELBOW,-70));
}

TEST_F(ControllerTest, UseElbowRestriction) {
  m_controller.parseChar('7');
  m_controller.parseChar('0');
  m_controller.parseChar('e');
  EXPECT_EQ(55, m_controller.curve(ELBOW).target());
}

TEST_F(ControllerTest, RestrictShoulder) {
  m_controller.curve(ELBOW).setPos(0);
  EXPECT_EQ( 45, m_controller.limitArm(SHOULDER, 70));
  EXPECT_EQ(-45, m_controller.limitArm(SHOULDER,-70));
}

TEST_F(ControllerTest, RestrictShoulderRelativeToElbow) {
  EXPECT_EQ( 25, m_controller.limitArm(SHOULDER, 70));
  EXPECT_EQ(-65, m_controller.limitArm(SHOULDER,-70));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
