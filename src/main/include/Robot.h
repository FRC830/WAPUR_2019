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

    // pin const declarations
    const int LEFT_FRONT_PIN = 3;
    const int LEFT_BACK_PIN = 1;
    const int RIGHT_BACK_PIN = 6;
    const int RIGHT_FRONT_PIN = 5;
    const int DEVICE_ID = 2;
    //Replace with real solonoid pin numbers later
    const int LEFT_SOLONOID_PIN = 0;                  
    const int RIGHT_SOLONOID_PIN = 0;
    const int CENTER_SOLONOID_PIN = 0;
    // Solonoids

    frc::Solenoid leftPiston{LEFT_SOLONOID_PIN};
    frc::Solenoid rightPiston{RIGHT_SOLONOID_PIN};
    frc::Solenoid punchPiston{CENTER_SOLONOID_PIN};
    // declare the motors objects
    WPI_TalonSRX left_back{LEFT_BACK_PIN};
    WPI_TalonSRX right_back{RIGHT_BACK_PIN};
    WPI_TalonSRX left_front{LEFT_FRONT_PIN};
    WPI_TalonSRX right_front{RIGHT_FRONT_PIN};
    Toggle isGrabbing{false};

    frc::I2C arduino{frc::I2C::Port::kOnboard, 4};

    // interate through all the motors objects and configure their values



    // declare the mecnum drive using the motor values
    // drive.setTargetVelocity(10)?
    frc::XboxController pilot{0};
    frc::XboxController copilot{1};
    static constexpr double sensitivity = 0.1;
    static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
    static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;
    static constexpr double DEADZONE_THRESHOLD = 0.1;

   private:
};