/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.OI;
import frc.robot.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Joystick;
/**
 *
 */
public class DriveTrain extends Subsystem {
	private boolean inReverseMode = false;//If this is true then the back of the robot will be considered the front
	
	public DriveUnit rightUnit;
	public DriveUnit leftUnit;
	private static double distanceBetweenWheels = 22.875/12.0; //feet
	private LinearMap power_RPM;
	//Variables for driving in straight line
	double linePFactor = 0.05;
	double lineAllowableAngleError = 0.1;
	double lineMaxMultiplyNumber = 3;
	double maxSpeedFPS = 8;
	
	/**
	 * The drivetrain class with methods needed for driving the robot
	 * @param rightUnit The Drive Unit for the right side of the robot
	 * @param leftUnit The Drive Unit for the left side of the robot
	 */
	public DriveTrain(DriveUnit rightUnit, DriveUnit leftUnit) {
		this.rightUnit = rightUnit;
		this.leftUnit = leftUnit;
		this.power_RPM = new LinearMap(0,0,1,1000);
	}
	
	/**
	 * With the given information this method will calculate the appropriate speeds that the motors on the left and right side of the robot will need to rotate at to maintain a straight line in the direction specified
	 * @param desiredSpeed A number that will be multiplied by the final result to increase the speeds of the motors
	 * @param desiredHeading The direction in Degrees that you want the robot to face when driving in a line
	 * @param currentLeftMotorSpeed The current speed of the motors on the left side of the robot
	 * @param currentRightMotorSpeed The current speed of the motors on the right side of the robot
	 * @param currentHeading The current heading of the robot in degrees (Must be an angle between -180 and +180 where a positive angle is a clockwise turn
	 * @return A double array with 2 indexes containing the new speeds that should be applied to the left and right motors of the robot, The first index is the new speed for the Left Drive Unit
	 */
	public double[] calculateMotorSpeedsForStraightLine(double desiredSpeed, double desiredHeading, double currentLeftMotorSpeed, double currentRightMotorSpeed, double currentHeading) {
		linePFactor = Robot.prefs.getDouble("Straight line P Factor",  0.05);
		lineMaxMultiplyNumber = Robot.prefs.getDouble("Line Max Multiply Number",  3);
		
		double newRightSpeed = currentRightMotorSpeed;
		double newLeftSpeed = currentLeftMotorSpeed;
		
		double currentAngle = currentHeading;
		double angleError = Degrees.subtract(currentAngle, desiredHeading);
		
		boolean isErrorNegative = angleError < 0;
		
		angleError = Math.sqrt(Math.abs(angleError));
		
		if (isErrorNegative) {
			angleError = -angleError;
		}
		
		double multiplyNumber = angleError * linePFactor;
		
		//Make sure the number is not too large or too small
		multiplyNumber = Math.min(lineMaxMultiplyNumber, Math.max(multiplyNumber, -lineMaxMultiplyNumber));
				
		SmartDashboard.putNumber("CurrentAngle", currentAngle);
		SmartDashboard.putNumber("Desired angle", desiredHeading);
		SmartDashboard.putNumber("Turn Error", angleError);
		
		
		
		newLeftSpeed = (1 - multiplyNumber) * desiredSpeed;
		newRightSpeed = (1 + multiplyNumber) * desiredSpeed;
		
		
		double[] newWheelSpeeds = {newLeftSpeed, newRightSpeed};
		
		SmartDashboard.putNumber("Requested Left Speed",  newLeftSpeed);
		SmartDashboard.putNumber("Requested Right Speed",  newRightSpeed);
		
		return newWheelSpeeds;
	}
	
	/**
	 * Makes the robot drive in the direction given
	 * @param directionAngle The direction in degrees that you want the robot to drive in
	 * @param speed The speed you want the robot to go at
	 * @param currentHeading The current heading of the robot in degrees
	 */
	public void driveInDirection(double directionAngle, double speed, double currentHeading) {
		double leftMotorSpeed = leftUnit.getVelocityFPS();
		double rightMotorSpeed = rightUnit.getVelocityFPS();
		
		double [] newMotorSpeeds = calculateMotorSpeedsForStraightLine(speed, directionAngle, leftMotorSpeed, rightMotorSpeed, currentHeading);
		
		double throttle = OI.joy.getThrottle();
		
		if (throttle > 0.5) {
			newMotorSpeeds[0] = -newMotorSpeeds[0];
			newMotorSpeeds[1] = -newMotorSpeeds[1];
		}
		
		leftUnit.setVelocityFPS(newMotorSpeeds[0]);
		rightUnit.setVelocityFPS(newMotorSpeeds[1]);
	}
	
	/**
	 * Gets the average distance that both sides of the robot have traveled
	 * @return Distance that the robot has traveled in feet
	 */
	
	public void driveWithJoystick(double x, double y, double throttle) {		
		
		//If the system is in reverse mode
		if (inReverseMode) {
			//Reverse the inputs
			y = -y;
		}
		double T=(throttle - 1)/-2;
		double X=(x*T);
		double Y=y;
		double V=(100-Math.abs(X))*(Y/100)+Y;
		double W=(100-Math.abs(Y))*(X/100)+X;
		double L=(V-W)/2;
		double R=(V+W)/2;
		
		L = L*20;
		R = R*20;
		
		boolean leftSpeedNegative = L < 0;
		boolean rightSpeedNegative = R < 0;
		
		R = Math.min(Math.abs(R),  maxSpeedFPS);
		L = Math.min(Math.abs(L),  maxSpeedFPS);
		
		if (leftSpeedNegative) {
			L = -L;
		}
		
		if (rightSpeedNegative) {
			R = -R;
		}
		
		leftUnit.setVelocityFPS(L);//The 30 increases speed
		rightUnit.setVelocityFPS(R);
		
		SmartDashboard.putNumber("Requested Left Speed",  L*20);
		SmartDashboard.putNumber("Requested Right Speed",  R*20);
	}
	
	public void driveUnitAtPercentSpeed(double speed) {
		leftUnit.setPercentSpeed(speed);
		rightUnit.setPercentSpeed(speed);
	}
	
	//Sets whether the robot is in reverse mode
	public void setReverseMode(boolean inReverse) {
		inReverseMode = inReverse;
	}
	
	//Returns true if the robot is in reverse mode
	public boolean getInReverseMode() {
		return inReverseMode;
	}
	
	public void stopDrive() {
		leftUnit.setPercentSpeed(0);
		rightUnit.setPercentSpeed(0);
	}
	
	public void setPercentSpeed(double voltage){
		leftUnit.setPercentSpeed(voltage);
		rightUnit.setPercentSpeed(voltage);
	}
	
	public double getLeftVoltage() {
		return leftUnit.getVoltage();
	}
	
	public double getRightVoltage() {
		return rightUnit.getVoltage();
	}
	
	public void driveToDistance(double feet){
		rightUnit.setFeet(feet);
		leftUnit.setFeet(feet);
	}
	
	public double getRightDistanceFeet() {
		return rightUnit.getFeet();
	}
	
	public double getLeftDistanceFeet() {
		return leftUnit.getFeet();
	}
	/**
	 * Drives the whole unit to the number of rotation specified
	 * @param numRotations The number of rotations the motor will complete
	 */
	public void driveUnitToRotation(double numberRotations) {
		rightUnit.setRotations(numberRotations);
		leftUnit.setRotations(numberRotations);
	}
	
	
	/**
	 * Turns the robot to the specified number of rotations of the motor, positive is to the right and negative is to the left
	 * @param rotations The number of motor rotations
	 */
	public void turnUnitToRotation(double rotations) {
		leftUnit.setRotations(rotations);
		rightUnit.setRotations(-rotations);
	}
	
	/**
	 * @return The number of rotations the rightUnit has completed
	 */
	public double getRightUnitRotations() {
		return rightUnit.getRotations();
	}
	
	/**
	 * @return The number of rotations the leftUnit has completed
	 */
	public double getLeftUnitRotations() {
		return leftUnit.getRotations();
	}
	
	/**
	 * @return The position of the right unit in encoder ticks
	 */
	public int getRightUnitPosition() {
		return rightUnit.getEncoderPosition();
	}
	
	/**
	 * @return The position of the left unit in encoder ticks
	 */
	public int getLeftUnitPosition() {
		return leftUnit.getEncoderPosition();
	}
	
	/**
	 * Resets the encoders of the drive unit so that their position reads 0
	 */
	public void zeroUnitEncoders() {
		rightUnit.zeroEncoder();
		leftUnit.zeroEncoder();
	}
	
	public int getRightUnitError() {
		return rightUnit.getPIDerror();
	}
	
	public int getLeftUnitError() {
		return leftUnit.getPIDerror();
	}
	
	/**
	 * Moves the whole unit at the speed specified
	 * @param fps The speed you want the motor to move at in Feet per second
	 */
	public void driveUnitAtFPS(double fps) {
		rightUnit.setVelocityFPS(fps);
		leftUnit.setVelocityFPS(fps);
	}
	
	public void driveUnitAtRPM(double RPM) {
		rightUnit.setVelocityRPM(RPM);
		leftUnit.setVelocityRPM(RPM);
	}
	
	public void driveRightUnitAtRPM(double RPM) {
		rightUnit.setVelocityRPM(RPM);
	}
	
	public void driveLeftUnitAtRPM(double RPM) {
		leftUnit.setVelocityRPM(RPM);
	}
	
	/**
	 * Makes the right drive unit run at the speed specified
	 * @param fps The speed in feet per second
	 */
	public void driveRightUnitAtFPS(double fps) {
		rightUnit.setVelocityFPS(fps);
	}
	
	/**
	 * Makes the left unit run at the speed specified
	 * @param fps The speed in feet per second
	 */
	public void driveLeftUnitAtFPS(double fps) {
		leftUnit.setVelocityFPS(fps);
	}
	
	
	/**
	 * Turns the unit at the given speed
	 * @param The speed in feet per second
	 */
	public void turnUnitAtFPS(double fps) {
		leftUnit.setVelocityFPS(fps);
		rightUnit.setVelocityFPS(-fps);
	}
	
	public void turnUnitAtDPS(double dps){
		setVelocityDPS(dps);
		setVelocityDPS(-dps);
	}
	
	/**
	 * @return The speed in feet per second the right unit is currently driving at
	 */
	public double getRightUnitFPS() {
		return rightUnit.getVelocityFPS() ;
	}
	
	/**
	 * @return The speed in feet per second the left unit is currently driving at
	 */
	public double getLeftUnitFPS() {
		return leftUnit.getVelocityFPS();
	}
	
	
	public void setUnitPID(double p, double i, double d, int allowableError) {
		rightUnit.configPID(p, i, d, allowableError);
		leftUnit.configPID(p,  i,  d,  allowableError);
	}
	

	@Override
	protected void initDefaultCommand() {
		//setDefaultCommand(new ArcadeDrive()); //this is removed because it is affecting autonomous as well
	}
	
	public void setBrakeMode(Boolean  brake){
		leftUnit.setBrakeMode(brake);
		rightUnit.setBrakeMode(brake);
	}
	
	/**
	 * 
	 * @param dps Degrees per second
	 */
	public void setVelocityDPS(double dps){
		double fps = DpsToFps(dps);
		turnUnitAtFPS(fps);
	}
	
	/**
	 * Gives the velocity of the left and right drive units as Degrees Per Second in a double array
	 * @return The velocity of both drive units, at the first index it gives the velocity of the left Drive unit
	 */
	public double[] getVelocityDPS(){
		double[] fps = {getLeftUnitFPS(), getRightUnitFPS()};
		double[] dps = {FpsToDps(fps[0]), FpsToDps(fps[1])};
		return dps;
	}
	
	/**
	 * Gives the velocity of the left and right drive units as Feet per Second in a double array
	 * @return The velocity of both drive units, at the first index it gives the Left Unit Velocity
	 */
	public double[] getVelocityFPS() {
		double[] fps = {getLeftUnitFPS(), getRightUnitFPS()};
		return fps;
	}
	
	/**
	 * 
	 * @param fps velocity in Feet per Second to convert to Degrees per Second
	 * @return Degrees per Second.
	 */
	public double FpsToDps(double fps){
		return fps * (360/(Math.PI*distanceBetweenWheels));
	}
	
	/**
	 * 
	 * @param dps Degrees Per Second
	 * @return Feet per Second
	 */
	public double DpsToFps(double dps){
		return dps * ((Math.PI*distanceBetweenWheels)/360.0);
	}
	public double getPowerFromRPM (double desiredRPM){
		return power_RPM.backward(desiredRPM);
	}
}