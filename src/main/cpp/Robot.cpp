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
    double rawX = fabs(pilot.GetX(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(LEFT);
    double rawY = fabs(pilot.GetY(LEFT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetY(LEFT);
    double turn = fabs(pilot.GetX(RIGHT)) < DEADZONE_THRESHOLD ? 0 : pilot.GetX(RIGHT);
    drive.DriveCartesian(rawX, -rawY, turn);


    /*Pistons*/
    // Shoot ball
    if (copilot.GetAButton()) {
      isGrabbing.toggle(false);
      punchPiston.Set(true);
    } else {
      punchPiston.Set(false);
    }

    isGrabbing.toggle(copilot.GetXButton());  // so they only have to press it once

    if (isGrabbing) {
      leftPiston.Set(true);
      rightPiston.Set(true);
    } else {
        leftPiston.Set(false);
        rightPiston.Set(false);
    }
    
    // LEDS
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