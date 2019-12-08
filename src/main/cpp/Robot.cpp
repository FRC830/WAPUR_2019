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
    left_front.Config_kP(0, 1);
    right_front.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    right_front.Config_kP(0, 1);
    right_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    right_back.Config_kP(0, 1);
    left_back.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 1, 0);
    left_back.Config_kP(0, 1);

    right_front.SetInverted(true);
    right_back.SetInverted(true);
    arm_motor.SetInverted(true);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
    // zero all sensors
    right_front.SetSelectedSensorPosition(0,0);
    right_back.SetSelectedSensorPosition(0,0);
    left_back.SetSelectedSensorPosition(0,0);
    left_front.SetSelectedSensorPosition(0,0);
    right_front.ConfigPeakOutputForward(.5);
    right_back.ConfigPeakOutputForward(.5);
    left_back.ConfigPeakOutputForward(.5);
    left_front.ConfigPeakOutputForward(.5);
}

void Robot::AutonomousPeriodic() {
    // between 7ft and 14ft = 10.5ft
    // distance to go / circum = # of ticks
    // MOVE ticks * ticks/rev
    const double position = ((14.8) / (6*3.1415)) * 4096; //ticks/rev
    SmartDashboard::PutNumber("right_front", right_front.GetSelectedSensorPosition());
    SmartDashboard::PutNumber("left_front", left_front.GetSelectedSensorPosition());
    SmartDashboard::PutNumber("left_back", left_back.GetSelectedSensorPosition());
    SmartDashboard::PutNumber("right_back", right_back.GetSelectedSensorPosition());

    if (right_back.GetSelectedSensorPosition() < position) {

    right_front.Set(motorcontrol::ControlMode::PercentOutput, 1);
    right_back.Set(motorcontrol::ControlMode::PercentOutput, 1);
    left_front.Set(motorcontrol::ControlMode::PercentOutput, 1);
    left_back.Set(motorcontrol::ControlMode::PercentOutput, 1);
    } else {
    right_front.Set(motorcontrol::ControlMode::PercentOutput, 0);
    right_back.Set(motorcontrol::ControlMode::PercentOutput, 0);
    left_front.Set(motorcontrol::ControlMode::PercentOutput, 0);
    left_back.Set(motorcontrol::ControlMode::PercentOutput, 0);

    }
}

void Robot::TeleopInit() {
    right_front.ConfigPeakOutputForward(1);
    right_back.ConfigPeakOutputForward(1);
    left_back.ConfigPeakOutputForward(1);
    left_front.ConfigPeakOutputForward(1);
}

void Robot::TeleopPeriodic() {

    HandleDrivetrain();
    HandleManipulator();
    HandleLEDStrip();

}
void Robot::HandleDrivetrain() {
    // Drivetrain
    // velocity setpoint in units/100ms
    // rev/min * unit/rev * min/100ms => units/100ms
    // 500 * 4096 * 1/600
    const double velocityConvertConstant = 500.0 * 4096 / 600;
    double right = fabs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT);
    double forward = fabs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : -pilot.GetY(LEFT);
    double turn = fabs(pilot.GetX(RIGHT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(RIGHT);
    // square outputs
    right *= fabs(right);
    forward *= fabs(forward);
    turn *= fabs(turn);
    double front_left_value = forward + turn + right;
    double front_right_value = forward - turn - right;
    double back_left_value = forward + turn - right;
    double back_right_value = forward - turn + right;
    // determine max
    double max_val = fabs(front_left_value);
    if (fabs(front_right_value) > max_val) { max_val = fabs(front_right_value); }
    if (fabs(back_left_value) > max_val) { max_val = fabs(back_left_value); }
    if (fabs(back_right_value) > max_val) { max_val = fabs(back_right_value); }
    // normalize all values to [-1, 1]
    if (max_val > 1) {
        front_left_value /= max_val;
        front_right_value /= max_val;
        back_left_value /= max_val;
        back_right_value /= max_val;
    }
    // multiply by velocity and set motors

    front_left_value *= velocityConvertConstant * sensitivity;
    back_left_value *= velocityConvertConstant * sensitivity;
    back_right_value *= velocityConvertConstant * sensitivity;
    front_right_value *= velocityConvertConstant * sensitivity;
    right_front.Set(motorcontrol::ControlMode::Velocity, front_right_value);
    right_back.Set(motorcontrol::ControlMode::Velocity, back_right_value);
    left_front.Set(motorcontrol::ControlMode::Velocity, front_left_value);
    left_back.Set(motorcontrol::ControlMode::Velocity, back_left_value);
    SmartDashboard::PutNumber("READ forward", forward);
    SmartDashboard::PutNumber("READ right", right);
    SmartDashboard::PutNumber("READ turn_scaled", turn);
    SmartDashboard::PutNumber("front_right motor output", front_right_value);
    SmartDashboard::PutNumber("front_left motor output", front_left_value);
    SmartDashboard::PutNumber("back_right motor output", back_right_value);
    SmartDashboard::PutNumber("back_left motor output", back_left_value);
}
void Robot::HandleManipulator() {
    // Shoot ball
    isGrabbing.toggle(copilot.GetXButton());  // so they only have to press it once
    
    if (copilot.GetAButton()) {
        timer.Start();
        isGrabbing = true;
    }
    double startTimer = 0.15;
    if (timer.Get() > startTimer + .2) {
        timer.Stop();
        timer.Reset();
    } else if (timer.Get() > startTimer) {
        punchPiston.Set(true);
    } else {
        punchPiston.Set(false);
    }
    // testPunch.toggle(copilot.GetYButton());
    // if (testPunch) {
    //     punchPiston.Set(true);
    // } else {
    //     punchPiston.Set(false);
    // }
    if (isGrabbing) {
        grabPiston.Set(true);
    } else {
        grabPiston.Set(false);
    }
    
    double change = fabs(copilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : copilot.GetY(LEFT);
    change *= fabs(change) * sensitivity;
    arm_motor.Set(motorcontrol::ControlMode::PercentOutput, -change);
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