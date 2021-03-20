// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include <frc/SpeedControllerGroup.h>

class Robot : public frc::TimedRobot
{
public:
  double rAccel = 0, lAccel = 0;
  Robot()
  {
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
  }

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0)
    {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(0.5, 0.0);
    }
    else
    {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }

  void TeleopInit() override {
    lAccel = 0;
    rAccel = 0;
  }

  void TeleopPeriodic() override
  {
    lAccel = ((lAccel*99)+m_stick.GetY(lHand))/100;
    rAccel = ((rAccel*9)+m_stick.GetX(rHand))/10;
    if(m_stick.GetStickButton(lHand) || m_stick.GetStickButton(rHand) ||
        m_stick.GetAButton() || m_stick.GetBButton()){
      lAccel = 0;
      rAccel = 0;
    }

    m_robotDrive.ArcadeDrive(lAccel, rAccel);

  }

  void TestInit() override {}

  void TestPeriodic() override {}

private:
  // Robot drive system
  WPI_VictorSPX m_frontLeft{0};
  WPI_VictorSPX m_frontRight{1};
  WPI_VictorSPX m_rearLeft{2};
  WPI_VictorSPX m_rearRight{3};
  frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};
  frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};
  frc::DifferentialDrive m_robotDrive{m_right,m_left};

  frc::GenericHID::JoystickHand lHand = frc::GenericHID::kLeftHand;
  frc::GenericHID::JoystickHand rHand = frc::GenericHID::kRightHand;

  frc::XboxController m_stick{0};
  frc::LiveWindow &m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
