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

    frc::Solonoid leftPiston{LEFT_SOLONOID_PIN};
    frc::Solonoid rightPiston{RIGHT_SOLONOID_PIN};
    frc::Solonoid punchPiston{CENTER_SOLONOID_PIN};
    // declare the motors objects
    WPI_TalonSRX LEFT_BACK{LEFT_BACK_PIN};
    WPI_TalonSRX RIGHT_BACK{RIGHT_BACK_PIN};
    WPI_TalonSRX LEFT_FRONT{LEFT_FRONT_PIN};
    WPI_TalonSRX RIGHT_FRONT{RIGHT_FRONT_PIN};
    Toggle IsGrabbing{false};

    frc::I2C arduino{frc::I2C::Port::kOnboard, 4};

    // motors array
    WPI_TalonSRX motors[4] = {LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK};

    // interate through all the motors objects and configure their values
    for (int i = 0; i < 4; i++) {
        motors[i].ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
        motors[i].Config_kP(1, 1, 0);
        motors[i].Config_kI(1, 0, 0);
        motors[i].Config_kD(1, 0, 0);
    }


    // declare the mecnum drive using the motor values
    frc::MecanumDrive drive{LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK};
    // controls
    // drive.setTargetVelocity(10)?
    frc::XboxController pilot{0};
    frc::XboxController copilot{1};
    static const frc::GenericHID::JoystickHand LEFT = frc::GenericHID::kLeftHand;
    static const frc::GenericHID::JoystickHand RIGHT = frc::GenericHID::kRightHand;
    static constexpr double DEADZONE_THRESHOLD = 0.1;

   private:
};