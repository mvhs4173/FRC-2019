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

  public enum ClawPosition {
    CLAW_UP,
    CLAW_DOWN,
    IN_BETWEEN,
    INVALID;
  }

  public ClawSubsystem() {
    this.gripMotor = Hardware.clawGripMotor;
    this.angleMotor = Hardware.clawAngleMotor;
    this.leftIntakeMotor = Hardware.clawLeftIntake;
    this.rightIntakeMotor = Hardware.clawRightIntake;
    this.clawUpLimit = Hardware.clawUpLimitSwitch;
    this.clawDownLimit = Hardware.clawDownLimitSwitch;
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
   * Gets the position of the claw
   * @return A ClawPosition Enum describing the position of the claw
   */
  public ClawPosition getClawPosition() {
    ClawPosition position = ClawPosition.IN_BETWEEN;

    if (this.clawUpLimit.isTriggered()) {
      position = ClawPosition.CLAW_UP;
    }else if(this.clawDownLimit.isTriggered()) {
      position = ClawPosition.CLAW_DOWN;
    }else if(!this.clawDownLimit.isTriggered() && !this.clawDownLimit.isTriggered()){
      position = ClawPosition.IN_BETWEEN;
    }else {
      position = ClawPosition.INVALID;
    }

    return position;
  }
}