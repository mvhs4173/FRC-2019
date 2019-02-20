/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Hardware;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class LinearSlide extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public enum SlidePosition {
    LOW,
    MEDIUM,
    HIGH,
    CONVERT,
    INTAKE;
  }

  public static final int highPosition = 3800;
  public static final int mediumPosition = 2000;
  public static final int lowPosition = 0;
  public static final int intakePosition = 500;
  public static final int hatchLowPosition = 500;
  public static final int hatchMediumPosition = 2500;
  public static final int hatchHighPosition = 3800;

  public static final int allowableError = 100;



  private MotorController lifter,
                          breakMotor;

  public LinearSlide(){
    this.lifter = Hardware.linearSlideLeftLift;
    breakMotor = Hardware.linearSlideBreak;
    lifter.resetEncoder();
  }

  public int getPosition() {
    return -lifter.getEncoderPosition();//Negate it so we get positive position when we go up 
  }

  public boolean getQuadPinState() {
    return lifter.getQuadAState();
  }

  /**
   * Gets the position of the slide as an enum
   */
  public SlidePosition getSlidePosition() {
    SlidePosition position = SlidePosition.LOW;
    
    if (OI.slideMedium.get()) {
      position = SlidePosition.MEDIUM;
    }else if (OI.slideHigh.get()) {
      position = SlidePosition.HIGH;
    }

    return position;
  }

  /**
  * Sets the speed of the lifter motors
  * @param rpm The speed in RPMs
  */
  public void setLifterSpeedRPM(double rpm) {
    lifter.setVelocityRPM(rpm);
  }

  /**
  * Gets the current speed of the motor
  * @return The speed of the motor in RPMs
  */
  public double getLifterSpeedRPM() {
    return lifter.getVelocityRPM();
  }

    /**
    * Resets the encoder position of the slide to 0
    */
  public void resetSlidePosition() {
    lifter.resetEncoder();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  } 

  public void stopBreak(){
    breakMotor.setPercentSpeed(0);
  }

  public boolean checkBrake() {
    return breakMotor.getForwardLimitSwitchTriggered();
  }

  public void setBreakPower(double power){
    breakMotor.setPercentSpeed(power);
  }

  public double getAmpUsage() {
    return lifter.getAmps();
  }

  public void stopSlide(){
    lifter.setPercentSpeed(0);
  }
}
