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

/**
 * Add your docs here.
 */
public class LinearSlide extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private MotorController lifter;

  public LinearSlide(){
    this.lifter = Hardware.linearSlideLeftLift;
  }

public int getPosition() {
  return lifter.getEncoderPosition(); 
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  
  }
}
