// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/Joystick.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/PIDController.h>

#include <ctre/phoenix/sensors/CANCoder.h>

#include <rev/CANSparkMax.h>

auto xy_speed = 1.0_mps;
auto rot_speed = 1.0_rad_per_s;

// I guess full power when we're 90 degrees off?
const double ARBITRARY_VALUE = 1.0 / 90.0;

frc::Joystick leftStick{0};
frc::Joystick rightStick{1};

// Locations for the swerve drive modules relative to the robot center.
frc::Translation2d frontLeftLocation{9.5_in, 9.5_in};
frc::Translation2d frontRightLocation{9.5_in, -9.5_in};
frc::Translation2d backLeftLocation{-9.5_in, 9.5_in};
frc::Translation2d backRightLocation{-9.5_in, -9.5_in};

// Creating my kinematics object using the module locations.
frc::SwerveDriveKinematics<4> kinematics{frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

frc2::PIDController flAngleController{1.0, 0.0, 0.0};
frc2::PIDController frAngleController{1.0, 0.0, 0.0};
frc2::PIDController blAngleController{1.0, 0.0, 0.0};
frc2::PIDController brAngleController{1.0, 0.0, 0.0};

rev::CANSparkMax flMotor{25, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax frMotor{23, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax blMotor{21, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
rev::CANSparkMax brMotor{27, rev::CANSparkMaxLowLevel::MotorType::kBrushless};

rev::SparkMaxAbsoluteEncoder flEncoder = flMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
rev::SparkMaxAbsoluteEncoder frEncoder = frMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
rev::SparkMaxAbsoluteEncoder blEncoder = blMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
rev::SparkMaxAbsoluteEncoder brEncoder = brMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);




void Robot::RobotInit() {

}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::ChassisSpeeds speeds{xy_speed * rightStick.GetX(), xy_speed * rightStick.GetY(), rot_speed * leftStick.GetX()};
  auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

  double flValue = flAngleController.Calculate(flEncoder.GetPosition(), (double)fl.angle.Degrees());
  double frValue = frAngleController.Calculate(frEncoder.GetPosition(), (double)fr.angle.Degrees());
  double blValue = blAngleController.Calculate(blEncoder.GetPosition(), (double)bl.angle.Degrees());
  double brValue = brAngleController.Calculate(brEncoder.GetPosition(), (double)br.angle.Degrees());

  flMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)

  flMotor.Set(flValue * ARBITRARY_VALUE);
  frMotor.Set(frValue * ARBITRARY_VALUE);
  blMotor.Set(blValue * ARBITRARY_VALUE);
  brMotor.Set(brValue * ARBITRARY_VALUE);
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
