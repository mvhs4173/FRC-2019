/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawPosition;

/**
 * An example command.  You can replace me with your own command.
 */
 public class MoveClawUpNoLimit extends Command {
  private boolean isFinished = false;

  private ClawSubsystem claw;
  // Limit switches for clawMotor and angleMotor

  public MoveClawUpNoLimit() {
    // Use requires() here to declare subsystem dependencies
    requires(Hardware.claw);
    this.claw = Hardware.claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //claw.disableUpperLimit();
    claw.raiseClawSlow();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    claw.raiseClawSlow();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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