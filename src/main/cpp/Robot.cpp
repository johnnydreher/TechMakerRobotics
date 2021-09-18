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
<<<<<<< HEAD
#include <frc/PowerDistributionPanel.h>
#include <frc/PWM.h>
=======
#include <iostream>
#include <frc/PowerDistributionPanel.h>
using namespace std;
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60

const static double kP = 0.03f;
const static double kI = 0.00f;
const static double kD = 0.00f;
const static double kF = 0.00f;

const static double kToleranceDegrees = 2.0f;

class Robot : public frc::TimedRobot
{
public:
  double rAccel = 0, lAccel = 0;
<<<<<<< HEAD
  double accel = 0;
  AutonomousState_t state = stoped;
  double AutonomousDistance = 0;
  int AutonomousCount = 0;
  const double largePath = 300.0;
  const double smallPath = 100.0;
  frc::PWM ledR{0};
  frc::PWM ledG{1};
  frc::PWM ledB{2};
=======
  double accel = -1;
  
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
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
    compressor.Start();
    target.Set(frc::DoubleSolenoid::kOff);
    intakeDown.Set(frc::DoubleSolenoid::kOff);
<<<<<<< HEAD
=======
    m_middleShooter.SetInverted(1);
    frc::CameraServer::GetInstance()->StartAutomaticCapture();
    frc::CameraServer::GetInstance()->SetSize(frc::CameraServer::kSize640x480);
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
  }

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
    m_robotDrive.TankDrive(accel, accel);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
  }

  void AutonomousPeriodic() override
  {
    
    frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
    frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
    frc::SmartDashboard::PutNumber("Aceleração esquerda", lAccel);
    frc::SmartDashboard::PutNumber("Aceleração Direita", rAccel);
<<<<<<< HEAD
    frc::SmartDashboard::PutNumber("Aceleração", accel);
    frc::SmartDashboard::PutNumber("Estado", state);
    frc::SmartDashboard::PutNumber("angulo", ahrs->GetAngle());
    frc::SmartDashboard::PutNumber("Velocidade X", ahrs->GetVelocityX());
    frc::SmartDashboard::PutNumber("Velocidade Y", ahrs->GetVelocityY());
    frc::SmartDashboard::PutNumber("Velocidade Z", ahrs->GetVelocityZ());
    frc::SmartDashboard::PutNumber("Trajetorias", AutonomousCount);

    /*if (state == forwarding)
=======
    if (m_encoderLeft.GetDistance() > 590.0 || m_encoderRight.GetDistance() > 590.0)
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
    {

      m_encoderLeft.Reset();
      m_encoderRight.Reset();
      accel = -0.5;
    }
    else if (m_encoderLeft.GetDistance() > 580.0 || m_encoderRight.GetDistance() > 580.0)
    {
<<<<<<< HEAD
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
=======
      accel = 0;
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
    }
    else if (m_encoderLeft.GetDistance() > 400.0 || m_encoderRight.GetDistance() > 400.0)
    {
<<<<<<< HEAD
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
=======
      accel = 0.5;
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
    }
    else if (m_encoderLeft.GetDistance() > 50.0 || m_encoderRight.GetDistance() > 50.0)
    {
      accel = 1.0;
    }

<<<<<<< HEAD
      m_robotDrive.ArcadeDrive(0, 0);
    }*/
=======
    if (m_encoderLeft.GetDistance() < -590.0 || m_encoderRight.GetDistance() < -590.0)
    {
      accel = 0;
    }
    else if (m_encoderLeft.GetDistance() < -580.0 || m_encoderRight.GetDistance() < -580.0)
    {
      accel = 0.25;
    }
    else if (m_encoderLeft.GetDistance() < -400.0 || m_encoderRight.GetDistance() < -400.0)
    {
      accel = -0.5;
    }
    else if (m_encoderLeft.GetDistance() < -50.0 || m_encoderRight.GetDistance() < -50.0)
    {
      accel = -1.0;
    }

    m_robotDrive.TankDrive(accel, accel);
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
  }

  void TeleopInit() override
  {
    lAccel = 0;
    rAccel = 0;
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
  }

  void TeleopPeriodic() override
  {
<<<<<<< HEAD
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
=======
   
   
   
   frc::SmartDashboard::PutNumber("Encoder Esquerda", m_encoderLeft.GetDistance());
   frc::SmartDashboard::PutNumber("Encoder Direita", m_encoderRight.GetDistance());
    if(m_pdp.GetVoltage()>Tmax)
    {
     accel = 0.7;
    }
    else if(m_pdp.GetVoltage()<Tmin)
    {
    frc::DriverStation::ReportError("Low Voltage");
    accel = 1;
    }
    else
    {
      accel = (Tmax - m_pdp.GetVoltage())/1000 + 0.8;
    }
    if(m_stick.GetStickButtonPressed(lHand))
    {
      if(shooter.Get()==0)
        shooter.Set(0.7);
      else
        shooter.Set(0);
    }
    if(m_stick.GetTriggerAxis(lHand)>0.5 || m_stick.GetTriggerAxis(rHand)>0.5)
    {
      conveyor.Set(1.0);
    }
    else{
      conveyor.Set(0);
    }
    if(m_stick.GetAButton())
    {
      if(intake.Get()==0)
        intake.Set(1.0);
      else
        intake.Set(0);
    }
    
    
    if(m_stick.GetBButton()){
      ahrs->ZeroYaw();
      while(ahrs->GetAngle()<100)
      {
         m_robotDrive.ArcadeDrive(0,0.75);
      }
      while(ahrs->GetAngle()<175)
      {
         m_robotDrive.ArcadeDrive(0,0.3);
      }
      m_timer.Reset();
      m_timer.Start();
      while(ahrs->GetAngle()<180)
      {
        m_robotDrive.ArcadeDrive(0,-0.3);
        if(m_timer.Get()>0.5)
        break;
      }
      m_robotDrive.ArcadeDrive(0,0);
    }
    
    if(m_stick.GetXButton())
    {
          m_robotDrive.ArcadeDrive(1.0,0);

    }
    else
        m_robotDrive.ArcadeDrive(-m_stick.GetY(lHand)*accel,(m_stick.GetX(rHand)*(accel-0.2)));
    
    if(m_stick.GetBButton())
    {
      m_robotDrive.ArcadeDrive(0, 1.0);
    }
    if(m_stick.GetBumper(lHand))
      intakeDown.Set(frc::DoubleSolenoid::kForward);
    else if(m_stick.GetBumper(rHand))
      intakeDown.Set(frc::DoubleSolenoid::kReverse);
    else
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
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
<<<<<<< HEAD
    

    if (m_stick.GetXButtonPressed())
=======
     if(m_stick.GetPOV()==0)
      target.Set(frc::DoubleSolenoid::kForward);
    else if(m_stick.GetPOV()==180)
      target.Set(frc::DoubleSolenoid::kReverse);
    else
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
    {
      if (ledB.GetRaw() > 0)
        ledB.SetRaw(0);
      else
        ledB.SetRaw(30000);
    }
<<<<<<< HEAD
=======
   
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
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
<<<<<<< HEAD
  WPI_VictorSPX m_conveyorUp{3};
  WPI_VictorSPX m_conveyorDown{0};
  WPI_VictorSPX m_leftShooter{2};
  WPI_VictorSPX m_rightShooter{1};
  rev::CANSparkMax intake{8, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::SpeedControllerGroup shooter{m_leftShooter, m_rightShooter};
  frc::SpeedControllerGroup conveyor{m_conveyorDown, m_conveyorUp, intake};
=======
  WPI_VictorSPX m_middleConveyor{0};
  WPI_VictorSPX m_middleShooter{3};
  WPI_VictorSPX m_leftShooter{2};
  WPI_VictorSPX m_rightShooter{1};
  frc::SpeedControllerGroup shooter{m_leftShooter, m_rightShooter};
  rev::CANSparkMax intake{8,rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::SpeedControllerGroup conveyor{m_middleConveyor, m_middleShooter, intake};
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
  AHRS *ahrs;
  int autoLoopCounter;
  const float Tmax = 12.5;
  const float Tmin = 10.5;
  frc::Compressor compressor;
<<<<<<< HEAD
  frc::DoubleSolenoid target{2, 3};
  frc::DoubleSolenoid intakeDown{0, 1};
=======
  frc::DoubleSolenoid target{2,3};
  frc::DoubleSolenoid intakeDown{0,1};
>>>>>>> 2c5ca29f0b6e00517d8bcde19506621f59883f60
  frc::PowerDistributionPanel m_pdp;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
 
  return frc::StartRobot<Robot>();
}
#endif