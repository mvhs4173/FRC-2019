/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;

public class GripHatch extends Command {
  private ClawSubsystem claw;
  boolean isDone = false;

  int closeGripperPosition = -100;
  int targetPosition = 10;
  int allowableError = 15;

  double pFactor = 0.01;

  public GripHatch() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Hardware.claw);
    this.claw = Hardware.claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    claw.setGripperSpeed(100);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    claw.setGripperSpeed(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    claw.setGripperSpeed(0.0);
  }
}
