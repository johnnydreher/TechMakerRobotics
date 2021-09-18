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
#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include "cameraserver/CameraServer.h"
#include <frc/PowerDistributionPanel.h>
#include <frc/PWM.h>

const static double kP = 0.03f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;

const static double kToleranceDegrees = 2.0f;

class Robot : public frc::TimedRobot
{
public:
  typedef enum AutonomousState
  {
    forwarding,
    lefting,
    righting,
    stoped
  } AutonomousState_t;
  double rAccel = 0, lAccel = 0;
  double accel = 0;
  AutonomousState_t state = stoped;
  double AutonomousDistance = 0;
  int AutonomousCount = 0;
  const double largePath = 300.0;
  const double smallPath = 100.0;
  frc::PWM ledR{0};
  frc::PWM ledG{1};
  frc::PWM ledB{2};
  Robot()
  {
    m_timer.Start();
    m_encoderLeft.SetDistancePerPulse(48.0 / 2048.0);
    m_encoderRight.SetDistancePerPulse(48.0 / 2048.0);
    ledR.SetRaw(30000);
    ledG.SetRaw(30000);
    ledB.SetRaw(30000);
    m_conveyorUp.SetInverted(1);
    try
    {

      ahrs = new AHRS(frc::SPI::Port::kMXP);
    }
    catch (std::exception &ex)
    {
      std::string what_string = ex.what();
      std::string err_msg("Error instantiating navX MXP:  " + what_string);
      const char *p_err_msg = err_msg.c_str();
      frc::DriverStation::ReportError(p_err_msg);
    }
    target.Set(frc::DoubleSolenoid::kOff);
    intakeDown.Set(frc::DoubleSolenoid::kOff);
  }

  void AutonomousInit() override
  {

    accel = 0.5;
    state = forwarding;
    m_robotDrive.TankDrive(accel, accel);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
    AutonomousDistance = largePath;
  }

  void AutonomousPeriodic() override
  {
    frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
    frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
    frc::SmartDashboard::PutNumber("Aceleração esquerda", lAccel);
    frc::SmartDashboard::PutNumber("Aceleração Direita", rAccel);
    frc::SmartDashboard::PutNumber("Aceleração", accel);
    frc::SmartDashboard::PutNumber("Estado", state);
    frc::SmartDashboard::PutNumber("angulo", ahrs->GetAngle());
    frc::SmartDashboard::PutNumber("Velocidade X", ahrs->GetVelocityX());
    frc::SmartDashboard::PutNumber("Velocidade Y", ahrs->GetVelocityY());
    frc::SmartDashboard::PutNumber("Velocidade Z", ahrs->GetVelocityZ());
    frc::SmartDashboard::PutNumber("Trajetorias", AutonomousCount);

    /*if (state == forwarding)
    {
      if (m_encoderLeft.GetDistance() > (AutonomousDistance * 0.98) || m_encoderRight.GetDistance() > (AutonomousDistance * 0.98))
      {

        state = lefting;
        accel = 0;
        ahrs->ZeroYaw();
        if (AutonomousCount < 2)
          AutonomousCount++;
        else
        {
          state = stoped;
        }
      }

      else if (m_encoderLeft.GetDistance() > (AutonomousDistance * 0.9) || m_encoderRight.GetDistance() > (AutonomousDistance * 0.9))
      {
        accel = 0.25;
      }

      else if (m_encoderLeft.GetDistance() > (AutonomousDistance * 0.1) || m_encoderRight.GetDistance() > (AutonomousDistance * 0.1))
      {
        accel = 1.0;
      }
      m_robotDrive.ArcadeDrive(accel, 0);
    }

    if (state == righting)
    {
      if (ahrs->GetAngle() < 75.0)
        m_robotDrive.TankDrive(0.4, -0.4);
      else if (ahrs->GetAngle() < 90.0)
        m_robotDrive.TankDrive(0.35, -0.35);

      else
      {
        if (AutonomousCount == 1)
          AutonomousDistance = smallPath;
        else
          AutonomousDistance = largePath;
        accel = 0.5;
        state = forwarding;
        m_robotDrive.TankDrive(accel, accel);
        m_encoderLeft.Reset();
        m_encoderRight.Reset();
      }
    }
    if (state == lefting)
    {
      if (ahrs->GetAngle() > -75.0)
        m_robotDrive.TankDrive(-0.4, 0.4);
      else if (ahrs->GetAngle() > -90.0)
        m_robotDrive.TankDrive(-0.35, 0.35);

      else
      {
        if (AutonomousCount == 1)
          AutonomousDistance = smallPath;
        else
          AutonomousDistance = largePath;
        accel = 0.5;
        state = forwarding;
        m_robotDrive.TankDrive(accel, accel);
        m_encoderLeft.Reset();
        m_encoderRight.Reset();
      }
    }
    if (state == stoped)
    {
      accel = 0;

      m_robotDrive.ArcadeDrive(0, 0);
    }*/
  }

  void
  TeleopInit() override
  {
    lAccel = 0;
    rAccel = 0;
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
    compressor.Start();
  }

  void TeleopPeriodic() override
  {
    frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
    frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
    frc::SmartDashboard::PutNumber("angulo", ahrs->GetAngle());
    frc::SmartDashboard::PutNumber("Velocidade X", ahrs->GetVelocityX());
    frc::SmartDashboard::PutNumber("Velocidade Y", ahrs->GetVelocityY());
    frc::SmartDashboard::PutNumber("Velocidade Z", ahrs->GetVelocityZ());

    m_robotDrive.ArcadeDrive(-m_stick.GetY(lHand), m_stick.GetX(rHand));
    if (m_stick.GetAButtonPressed())
    {
      if (ledR.GetRaw() > 0)
      {
        ledR.SetRaw(0);
      }
      else
      {
        ledR.SetRaw(30000);
      }
    }
    if (m_stick.GetBButtonPressed())
    {
      if (ledG.GetRaw() > 0)
      {
        ledG.SetRaw(0);
      }
      else
      {
        ledG.SetRaw(30000);
      }
    }
    

    if (m_stick.GetXButtonPressed())
    {
      if (ledB.GetRaw() > 0)
        ledB.SetRaw(0);
      else
        ledB.SetRaw(30000);
    }
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
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

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
  WPI_VictorSPX m_conveyorUp{3};
  WPI_VictorSPX m_conveyorDown{0};
  WPI_VictorSPX m_leftShooter{2};
  WPI_VictorSPX m_rightShooter{1};
  rev::CANSparkMax intake{8, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::SpeedControllerGroup shooter{m_leftShooter, m_rightShooter};
  frc::SpeedControllerGroup conveyor{m_conveyorDown, m_conveyorUp, intake};
  AHRS *ahrs;
  int autoLoopCounter;
  frc::Compressor compressor;
  frc::DoubleSolenoid target{2, 3};
  frc::DoubleSolenoid intakeDown{0, 1};
  frc::PowerDistributionPanel m_pdp;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
