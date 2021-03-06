/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.sound.sampled.Line;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LinearSlide;
import frc.robot.Hardware;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.LinearSlide.SlidePosition;

public class MoveLinearSlideToPosition extends Command {
  private LinearSlide linearSlide;

  private boolean isDone = false;
  private boolean activateBraking = false;

  //The encoder positions of all the heights we want the slide to be able to reach
  private int highPosition = LinearSlide.highPosition;
  private int mediumPosition = LinearSlide.mediumPosition;
  public  int lowPosition = LinearSlide.lowPosition;
  private int intakePosition = LinearSlide.intakePosition;

  public static final int allowableError = LinearSlide.allowableError;//The number of ticks that the slide can be off by while still having the target being considered reached

  private double pFactor = 5300;//P factor for Proportional loop for smooth movement of the slide

  BrakeLinearSlide activateBrake;

  private SlidePosition targetSlidePosition;
  private int targetSlidePositionTicks;

  SlidePosition startPosition;
  
  MoveClawUp clawUp;
  MoveClawDown clawDown;

  /**
   * Moves the slide to the specified position, can move to any of the levels on the rocket
   */
  public MoveLinearSlideToPosition(SlidePosition slidePosition) {
    startPosition = slidePosition;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Hardware.linearSlide);
    linearSlide = Hardware.linearSlide;

    targetSlidePosition = slidePosition;
    activateBrake = new BrakeLinearSlide();

    clawUp = new MoveClawUp();
    clawDown = new MoveClawDown();
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
    boolean hatchSwitchState = OI.switchToHatch.get();

    //Check if the claw needs to be moved up
    if (!hatchSwitchState && (targetSlidePosition == SlidePosition.HIGH || targetSlidePosition == SlidePosition.MEDIUM)) {
      //Make sure the command isn't currently running
      if (clawUp.isRunning() == false) {
        clawUp.start();
      }
    }
    if (targetSlidePosition == SlidePosition.INTAKE){
      //Make sure the command isn't running
      if (clawDown.isRunning() == false) {
        clawDown.start();
      }
    }


      if (startPosition == SlidePosition.CONVERT) {
        targetSlidePosition = linearSlide.getSlidePosition();
      }
  
      //If the switch to hatch button is triggered then change the positions
      if (hatchSwitchState) {
        lowPosition = LinearSlide.hatchLowPosition;
        mediumPosition = LinearSlide.hatchMediumPosition;
        highPosition = LinearSlide.hatchHighPosition;
      } else {
        lowPosition = LinearSlide.lowPosition;
        mediumPosition = LinearSlide.mediumPosition;
        highPosition = LinearSlide.highPosition;
      }
  
      //Determine the target position in ticks
      if (targetSlidePosition == SlidePosition.LOW) {
        targetSlidePositionTicks = lowPosition;
      }else if(targetSlidePosition == SlidePosition.INTAKE){
        targetSlidePositionTicks = intakePosition;
      } else if(targetSlidePosition == SlidePosition.MEDIUM) {
        targetSlidePositionTicks = mediumPosition;
      }else if (targetSlidePosition == SlidePosition.HIGH) {
        targetSlidePositionTicks = highPosition;
      }

      SmartDashboard.putString("Slide Position", targetSlidePosition.toString());
      SmartDashboard.putNumber("TargetPosition", targetSlidePositionTicks);

    //pFactor = Robot.prefs.getDouble("Linearslide pFactor", 0.01);//For tuning the pFactor

    int currentPosition = linearSlide.getPosition();//The current position of the linearSlide in ticks

    int error = targetSlidePositionTicks - currentPosition;//The difference between the target position and current position
    //Calculate new motor speed to reach target
    double newSpeed = error * pFactor;
    boolean speedIsNegative = newSpeed < 0;
    newSpeed = Math.max(Math.sqrt(Math.abs(newSpeed)), 1500);

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
