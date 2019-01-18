/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;

/**
 * An example command.  You can replace me with your own command.
 */
 public class MoveClawUp extends Command {
  private boolean finished = false;
  private ClawSubsystem clawSubsystem;
  // Positive position of the claw
  private int clawPositivePosition = 10;

  public MoveClawUp(ClawSubsystem clawSubsystem) {
    // Use requires() here to declare subsystem dependencies
    requires(clawSubsystem);
    this.clawSubsystem = clawSubsystem;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int clawEncoderPosition = clawSubsystem.getAngleMotorPositionRaw();
    clawSubsystem.clawOpen();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}