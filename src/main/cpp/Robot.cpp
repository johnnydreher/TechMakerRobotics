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
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/math>
#include <frc/DutyCycleEncoder.h>
class Robot : public frc::TimedRobot
{
public:
  double rAccel = 0, lAccel = 0;
  double accel = -1;
  Robot()
  {
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
    m_encoderLeft.SetDistancePerPulse(48.0 / 2048.0);
    m_encoderRight.SetDistancePerPulse(48.0 / 2048.0);
   
  }
  

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
    m_robotDrive.TankDrive(accel,accel);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
    accel = -0.5;
  }

  void AutonomousPeriodic() override
  {
     frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
     frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
     frc::SmartDashboard::PutNumber("Aceleração", accel);

     if (m_encoderLeft.GetDistance()>590.0 || m_encoderRight.GetDistance()>590.0)
     {
       accel = 0;
     }
     else if (m_encoderLeft.GetDistance()>580.0 || m_encoderRight.GetDistance()>580.0)
     {
        accel = 0.25;
     }
     else if (m_encoderLeft.GetDistance()>500.0 || m_encoderRight.GetDistance()>500.0)
     {
        accel = -0.5;
     }
     else if (m_encoderLeft.GetDistance()>100.0 || m_encoderRight.GetDistance()>100.0)
     {
       accel = -1.0;
     }
     
    
     m_robotDrive.TankDrive(accel,accel);

  }

  void TeleopInit() override {
    lAccel = 0;
    rAccel = 0;
     m_encoderLeft.Reset();
       m_encoderRight.Reset();
  }

  void TeleopPeriodic() override
  {
     frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
     frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
    lAccel = m_stick.GetY(lHand);
    rAccel = m_stick.GetX(lHand);
    if(m_stick.GetXButtonPressed())
    {
     m_encoderLeft.Reset();
       m_encoderRight.Reset();  
    }

    m_robotDrive.ArcadeDrive(lAccel,rAccel);

  }

  void TestInit() override {}

  void TestPeriodic() override {}

private:
  // Robot drive system
  WPI_VictorSPX m_frontLeft{4};
  WPI_VictorSPX m_frontRight{5};
  WPI_VictorSPX m_rearLeft{6};
  WPI_VictorSPX m_rearRight{7};
  frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};
  frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};
  frc::DifferentialDrive m_robotDrive{m_right,m_left};

  frc::GenericHID::JoystickHand lHand = frc::GenericHID::kLeftHand;
  frc::GenericHID::JoystickHand rHand = frc::GenericHID::kRightHand;

  frc::XboxController m_stick{0};
  frc::LiveWindow &m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
  //encoder esquerda
  // 9 - enc I
  // 8 - enc B
  // 7 - enc A
  // 6 - enc ABS
  //encoder direita
  // 5 - enc I
  // 4 - enc B
  // 3 - enc A
  // 2 - enc ABS
  
  frc::Encoder m_encoderLeft{8, 7};
  frc::Encoder m_encoderRight{4, 3};



};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
