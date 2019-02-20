/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.LinearSlide.SlidePosition;
import frc.robot.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public static Joystick joy = new Joystick(0);
	public static Joystick launchpad = new Joystick(1);
  public static Button slideUp = new JoystickButton(launchpad, 3),
        closeGripper = new JoystickButton(launchpad, 4),
        intakeCargo = new JoystickButton(launchpad, 1),
        expelCargo = new JoystickButton(launchpad, 2),
        angleCollectorDown = new JoystickButton(launchpad, 10),
        angleCollectorUp = new JoystickButton(launchpad, 15),
        resetEncoder = new JoystickButton(joy, 7),
        button12 = new JoystickButton(joy,12),
        stowClaw = new JoystickButton(launchpad, 11),
        slideLow = new JoystickButton(launchpad, 6),
        slideMedium = new JoystickButton(launchpad, 7),
        slideHigh = new JoystickButton(launchpad, 9),
        switchToHatch = new JoystickButton(launchpad, 13),
        intake = new JoystickButton(launchpad, 5);
  public OI (){
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
    slideUp.whenPressed(new MoveLinearSlideUp());
    closeGripper.whenPressed(new GripHatch());
    closeGripper.whenReleased(new StopGrabber());
    intakeCargo.whenPressed(new CollectBall());
    expelCargo.whenPressed(new ExpelCargo());
    angleCollectorDown.whenPressed(new MoveClawDown());
    angleCollectorUp.whenPressed(new MoveClawUp());
    //intakeCargo.whenReleased(new StopIntake());
    expelCargo.whenReleased(new StopIntake());
    slideUp.whenReleased(new StopLinearSlide());
    stowClaw.whenPressed(new MoveClawUpNoLimit());
    stowClaw.whenReleased(new StopClaw());
    slideLow.whenPressed(new UpThenBrake(SlidePosition.LOW));
    slideMedium.whenPressed(new UpThenBrake(SlidePosition.MEDIUM));
    slideHigh.whenPressed(new UpThenBrake(SlidePosition.HIGH));
    switchToHatch.whenPressed(new UpThenBrake(SlidePosition.CONVERT));
    switchToHatch.whenReleased(new UpThenBrake(SlidePosition.CONVERT));
    intake.whenPressed(new UpThenBrake(SlidePosition.INTAKE));
    resetEncoder.whenPressed(new ResetLinearEncoder());
  }
}
