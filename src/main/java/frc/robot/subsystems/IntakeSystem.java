package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Subsystem;


public class IntakeSystem extends Subsystem {
    private MotorController leftIntakeMotor;
    private MotorController rightIntakeMotor;

    public IntakeSystem() {
        this.leftIntakeMotor =  Hardware.clawLeftIntake;
        this.rightIntakeMotor = Hardware.clawRightIntake;
        leftIntakeMotor.disableReverseLimitSwitch();
        leftIntakeMotor.enableForwardLimitSwitch();
        rightIntakeMotor.setFollower(leftIntakeMotor);
        rightIntakeMotor.setDirection(InvertType.OpposeMaster);
    }

    public void initDefaultCommand() {

    }
    /**
   * Stops the intake motors
   */
  public void stopIntake() {
    leftIntakeMotor.setVelocityRPM(0);
  }

  public void intakeCargo(){
    leftIntakeMotor.setPercentSpeed(1);
  }

  public void expelCargo(){
    leftIntakeMotor.setPercentSpeed(-1);
  }

  
  public boolean cargoSwitchPressed(){
    return Hardware.linearSlideRightLift.getReverseLimitSwitchTriggered();
  }
  
}