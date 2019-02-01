/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
@SuppressWarnings("unused")
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	/*
	 * CAN ID for motors
	 * PWM 0-9
	 * DIGITAL 0-9
	 * ANALOG 0-3
	 */
	public static int FRONT_RIGHT_WHEEL = 42,
			FRONT_LEFT_WHEEL = 18,
			REAR_RIGHT_WHEEL = 16,
			REAR_LEFT_WHEEL = 17,
			COLLECTOR_LEFT_INTAKE = 40,
			COLLECTOR_RIGHT_INTAKE = 41,
			SLIDE_MOTOR = 15,
			HORIZONTAL_SLIDE_MOTOR = 14,
			SLIDE_LOW_SWITCH = 0,
			CLIMB_MOTOR = 43;
	
}
