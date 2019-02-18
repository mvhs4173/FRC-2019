/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Hardware;
import frc.robot.subsystems.LinearSlide;
import frc.robot.subsystems.Timer;

public class ReleaseLinearBrake extends Command {
  LinearSlide slide;

  public ReleaseLinearBrake() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    //requires(Hardware.linearSlide);
    slide = Hardware.linearSlide;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(slide.checkBrake()){
      slide.setBreakPower(-0.5);
      slide.setLifterSpeedRPM(1000);
    } else if(!slide.checkBrake()) {
      slide.stopBreak();
      slide.setLifterSpeedRPM(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !slide.checkBrake();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    slide.stopBreak();
    slide.setLifterSpeedRPM(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    slide.stopBreak();
    slide.setLifterSpeedRPM(0);
  }
}
