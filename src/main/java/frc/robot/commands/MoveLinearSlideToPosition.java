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
  private boolean activateBraking = false;

  //The encoder positions of all the heights we want the slide to be able to reach
  private int highPosition = LinearSlide.highPosition;
  private int mediumPosition = LinearSlide.mediumPosition;
  public  int lowPosition = LinearSlide.lowPosition;

  public static final int allowableError = LinearSlide.allowableError;//The number of ticks that the slide can be off by while still having the target being considered reached

  private double pFactor = 5300;//P factor for Proportional loop for smooth movement of the slide

  BrakeLinearSlide activateBrake;

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
    activateBrake = new BrakeLinearSlide();

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

    SmartDashboard.putNumber("TargetPosition", targetSlidePositionTicks);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    activateBraking = false;
    isDone = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    pFactor = Robot.prefs.getDouble("Linearslide pFactor", 0.01);//For tuning the pFactor

    int currentPosition = linearSlide.getPosition();//The current position of the linearSlide in ticks

    int error = targetSlidePositionTicks - currentPosition;//The difference between the target position and current position
    //Calculate new motor speed to reach target
    double newSpeed = error * pFactor;
    boolean speedIsNegative = newSpeed < 0;
    newSpeed = Math.max(Math.sqrt(Math.abs(newSpeed)), 900);

    if (speedIsNegative && newSpeed != 900) {
      newSpeed = -newSpeed;
    }

    if (targetSlidePositionTicks < currentPosition || targetSlidePositionTicks == 0) {
      newSpeed = 0.0;
    }

    SmartDashboard.putNumber("Linearslide Speed", newSpeed);
    SmartDashboard.putNumber("Linearslide Error", error);

    linearSlide.setLifterSpeedRPM(newSpeed);


    //Determine if the slide has reached the desired position
    if (Math.abs(error) <= allowableError && activateBraking == false) {
      activateBrake.start();
      activateBraking = true;
    }


    if (activateBrake.isFinished()) {
      linearSlide.stopSlide();
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
