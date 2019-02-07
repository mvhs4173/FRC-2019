/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class ClawSubsystem extends Subsystem {
  private MotorController gripMotor;
  private MotorController angleMotor;
  private MotorController leftIntakeMotor;
  private MotorController rightIntakeMotor;
  private LimitSwitch clawUpLimit,
                  clawDownLimit;
  private double clawSpeed = MotorController.getPowerFromRPM(1);
  private double origin = 0;

  public enum ClawPosition {
    CLAW_UP,
    CLAW_DOWN
  }

  public ClawSubsystem(MotorController gripMotor,
                       MotorController angleMotor,
                       MotorController leftIntakeMotor,
                       MotorController rightIntakeMotor,
                       LimitSwitch clawUpLimit,
                       LimitSwitch clawDownLimit) {
    this.gripMotor = gripMotor;
    this.angleMotor = angleMotor;
    this.leftIntakeMotor = leftIntakeMotor;
    this.rightIntakeMotor = rightIntakeMotor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
   * Stops the intake motors
   */
  public void stopIntake() {
    leftIntakeMotor.setVelocityRPM(0);
    rightIntakeMotor.setVelocityRPM(0);
  }

  /**
   * Raises the system
   */
  public void raiseClawSystem() {
    angleMotor.setPercentSpeed(clawSpeed);
  }

  /**
   * Lowers the system
   */
  public void lowerClawSystem() {
    angleMotor.setPercentSpeed(-clawSpeed);
  }

  /**
   * Stops the system from moving
   */
  public void stopClawSystem() {
    angleMotor.setPercentSpeed(0.0);
  }

  /**
   * Detects if the claw is in the up position
   */
  public boolean isClawUp() {
    return clawUpLimit.isTriggered();
  }

  /**
   * Detects if the claw is in the down position
   */
  public boolean isClawDown() {
    return clawDownLimit.isTriggered();
  }

  /**
   * Detects if the claw is in neither the up or down position
   */
  public boolean isClawInBetween() {
    return !clawUpLimit.isTriggered() && !clawDownLimit.isTriggered();
  }
}