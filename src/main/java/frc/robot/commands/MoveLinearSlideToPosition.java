/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LinearSlide;
import frc.robot.Hardware;
import frc.robot.Robot;

public class MoveLinearSlideToPosition extends Command {
  private LinearSlide linearSlide;

  private boolean isDone = false;

  //The encoder positions of all the heights we want the slide to be able to reach
  private int highPosition = 5000;
  private int mediumPosition = 2500;
  private int lowPosition = 1250;

  private double allowableSpeedError = 5;//The minimum that the motor has to be running at in order for the system to consider the target reached 
  private int allowableError = 100;//The number of ticks that the slide can be off by while still having the target being considered reached

  private double pFactor = 0.01;//P factor for Proportional loop for smooth movement of the slide


  private SlidePosition targetSlidePosition;
  private int targetSlidePositionTicks;

  public enum SlidePosition {
    LOW,
    MEDIUM,
    HIGH;
  }

  /**
   * Moves the slide to the specified position, can move to any of the levels on the rocket
   */
  public MoveLinearSlideToPosition(SlidePosition slidePosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Hardware.linearSlide);
    this.linearSlide = Hardware.linearSlide;

    this.targetSlidePosition = slidePosition;

    //Determine the target position in ticks
    switch(this.targetSlidePosition) {
      case LOW:
        targetSlidePositionTicks = lowPosition;
        break;
      case MEDIUM:
        targetSlidePositionTicks = mediumPosition;
        break;
      case HIGH:
        targetSlidePositionTicks = highPosition;
        break;
    }

    highPosition = Robot.prefs.getInt("High Position", 5000);//For tuning high position ticks
    mediumPosition = Robot.prefs.getInt("Medium Position", 2500);//For tuning medium position ticks
    lowPosition = Robot.prefs.getInt("Low Position", 1250);//For tuning low position ticks
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pFactor = Robot.prefs.getDouble("pFactor", 0.01);//For tuning the pFactor
    

    int currentPosition = linearSlide.getPosition();//The current position of the linearSlide in ticks
    SmartDashboard.putNumber("Linear slide Position", currentPosition);//For debugging
    
    int error = targetSlidePositionTicks - currentPosition;//The difference between the target position and current position
    //Calculate new motor speed to reach target
    double newSpeed = error * pFactor;

    linearSlide.setLifterSpeedRPM(newSpeed);

    double currentLifterSpeed = linearSlide.getLifterSpeedRPM();//The current speed of the lifter motor in RPMs

    //Determine if the slide has reached the desired position
    if (Math.abs(currentLifterSpeed) <= allowableSpeedError && Math.abs(error) <= allowableError) {
      linearSlide.setLifterSpeedRPM(0);//Stop the slide
      isDone = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    linearSlide.setLifterSpeedRPM(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    linearSlide.setLifterSpeedRPM(0);
  }
}
