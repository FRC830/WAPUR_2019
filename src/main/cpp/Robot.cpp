/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include "led_modes.h"

#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

void Robot::RobotInit() {
    left_front.ConfigFactoryDefault();
    right_front.ConfigFactoryDefault();
    right_back.ConfigFactoryDefault();
    left_back.ConfigFactoryDefault();

    left_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    left_front.Config_kP(1, 0, 0);
    right_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    right_front.Config_kP(1, 0, 0);
    right_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    right_back.Config_kP(1, 0, 0);
    left_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    left_back.Config_kP(1, 0, 0);
    // sparkMotor.RestoreFactoryDefaults();
    // auto encoder = sparkMotor.GetEncoder();
    // encoder.SetPosition(0);
    // auto PID_controller = sparkMotor.GetPIDController();
    // PID_controller.SetFeedbackDevice(encoder);
    // PID_controller.SetP(1);
    // PID_controller.SetI(0);
    // PID_controller.SetD(0);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

    // Drivetrain
    // velocity setpoint in units/100ms
    // rpm * units/rev * 600 100ms in every min
    const double velocityConvertConstant = 500.0 * 4096 / 600;
    double right = fabs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT);
    double forward = fabs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : -pilot.GetY(LEFT);
    double turn = fabs(pilot.GetX(RIGHT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(RIGHT);
    turn = turn * sensitivity;
    double front_left_value = forward + turn + right;
    double front_right_value = forward - turn - right;
    double back_left_value = forward + turn - right;
    double back_right_value = forward - turn + right;
    double max_val = fabs(front_left_value);
    if (fabs(front_right_value) > max_val) {
        max_val = fabs(front_right_value);
    }
    if (fabs(back_left_value) > max_val) { max_val = fabs(back_left_value); }
    if (fabs(back_right_value) > max_val) { max_val = fabs(back_right_value); }
    if (max_val > 1) {
        front_left_value /= max_val;
        front_right_value /= max_val;
        back_left_value /= max_val;
        back_right_value /= max_val;
    }
    // front_left_value *= velocityConvertConstant;
    // back_left_value *= velocityConvertConstant;
    // back_right_value *= velocityConvertConstant;
    // front_right_value *= velocityConvertConstant;
    // right_front.Set(motorcontrol::ControlMode::Velocity, front_right_value);
    // right_back.Set(motorcontrol::ControlMode::Velocity, back_right_value);
    // left_front.Set(motorcontrol::ControlMode::Velocity, front_left_value);
    // left_back.Set(motorcontrol::ControlMode::Velocity, back_left_value);
    right_front.Set(motorcontrol::ControlMode::PercentOutput, -front_right_value);
    right_back.Set(motorcontrol::ControlMode::PercentOutput, -back_right_value);
    left_front.Set(motorcontrol::ControlMode::PercentOutput, front_left_value);
    left_back.Set(motorcontrol::ControlMode::PercentOutput, back_left_value);

    SmartDashboard::PutNumber("READ forward", forward);
    SmartDashboard::PutNumber("READ right", right);
    SmartDashboard::PutNumber("READ turn_scaled", turn);
    SmartDashboard::PutNumber("front_right motor output", front_right_value);
    SmartDashboard::PutNumber("front_left motor output", front_left_value);
    SmartDashboard::PutNumber("back_right motor output", back_right_value);
    SmartDashboard::PutNumber("back_left motor output", back_left_value);
    HandleManipulator();
    HandleLEDStrip();

}
void Robot::HandleManipulator() {
    // Shoot ball
    isGrabbing.toggle(copilot.GetXButton());  // so they only have to press it once
    
    if (copilot.GetAButton()) {
        isGrabbing.toggle(false);
        punchPiston.Set(true);
    } else {
        punchPiston.Set(false);
    }


    if (isGrabbing) {
        leftPiston.Set(true);
        rightPiston.Set(true);
    } else {
        leftPiston.Set(false);
        rightPiston.Set(false);
    }
}
void Robot::HandleLEDStrip() {
    uint8_t led_mode = NONE;
    int pov = copilot.GetPOV();
    if (45 <= pov && pov <= 135) {
        // right
        DriverStation::Alliance allianceColor = DriverStation::GetInstance().GetAlliance();
        if (allianceColor == DriverStation::Alliance::kRed) {
            led_mode = RED_ALLIANCE;
        } else {
            led_mode = BLUE_ALLIANCE;
        }
    } else if (225 <= pov && pov <= 315) {
        // left
        led_mode = RATPACK;
    } else if (315 <= pov || (0 <= pov && pov <= 45)) {
        // up
        led_mode = SLOW_RAINBOW;
    } else if (135 <= pov && pov <= 225) {
        // down
        led_mode = FAST_RAINBOW;
    }
    // Alliance red -> yellow -> red
    if (led_mode != NONE) {
        arduino.WriteBulk(&led_mode, 1);
    }
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif