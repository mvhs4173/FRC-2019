/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.LinearSlide;
import frc.robot.Hardware;

public class MoveLinearSlideUp extends Command {
  private LinearSlide linearSlide;
  double lifterSpeedRPM = 50;

  /**
   * This command is used for manual movement of the linear slide by the driver
   * Moves slide at a constant speed
   */
  public MoveLinearSlideUp() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.linearSlide = Hardware.linearSlide;
    requires(Hardware.linearSlide);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    linearSlide.setLifterSpeedRPM(lifterSpeedRPM);
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
