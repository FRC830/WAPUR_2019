/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#pragma once

#include <string>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include "Toggle.h"
//Delete?
// #include <rev/CANSparkMax.h>

class Robot : public frc::TimedRobot {
   public:
      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopInit() override;
      void TeleopPeriodic() override;
      void TestPeriodic() override;
      void HandleLEDStrip();
      void HandleManipulator();
      void HandleDrivetrain();
      // pin const declarations
      const int LEFT_FRONT_PIN = 1;
      const int LEFT_BACK_PIN = 2;
      const int RIGHT_BACK_PIN = 3;
      const int RIGHT_FRONT_PIN = 4;
      const int ARM_MOTOR_PIN = 5; //change
      const int DEVICE_ID = 2;
      //Replace with real solonoid pin numbers later
      const int GRAB_SOLONOID_PIN = 5;
      const int CENTER_SOLONOID_PIN = 3;
      // Solonoids

      frc::Solenoid grabPiston{GRAB_SOLONOID_PIN};
      frc::Solenoid punchPiston{CENTER_SOLONOID_PIN};
      // declare the motors objects
      TalonSRX left_back{LEFT_BACK_PIN};
      TalonSRX right_back{RIGHT_BACK_PIN};
      TalonSRX left_front{LEFT_FRONT_PIN};
      TalonSRX right_front{RIGHT_FRONT_PIN};
      VictorSPX arm_motor{ARM_MOTOR_PIN};
      Toggle isGrabbing{false};
      Toggle testPunch{false};
      frc::Timer timer;
      frc::I2C arduino{frc::I2C::Port::kOnboard, 4};

      // interate through all the motors objects and configure their values



      // declare the mecnum drive using the motor values
      // drive.setTargetVelocity(10)?
      frc::XboxController pilot{0};
      frc::XboxController copilot{1};
      static constexpr double sensitivity = 1;
      static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
      static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;
      static constexpr double DEADZONE_THRESHOLD = 0.15;

   private:
};