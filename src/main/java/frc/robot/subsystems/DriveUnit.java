package frc.robot.subsystems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//Forward for the drive motors is the front of the robot

/**
 * A drive unit is two motors with one motor controller following the other one.
 * They are attatched to one gearbox.
 * There is a right and a left drive unit.
 */
public class DriveUnit {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private MotorController rearMotor,
			 frontMotor;
	
	UnitSide unitSide;
	private final double wheelDiameter = 6.0/12.0; //feet
	private final double ticksPerShaftRotation = 4096.0;
	
	private final double rpmToClicksPer100ms =  ticksPerShaftRotation/1.0 * 1.0/60.0 * 1.0/10.0;// rev/min = 4096 clicks/rev * 1min/60s * 1s/10 centi seconds
	private final double actualOverRequestedRPM =  0.85589;
	Preferences prefs;
	
	public enum UnitSide {
		RIGHT,
		LEFT;
	}
	
	/***
	 * 
	 * @param frontCanID The id of the front drive motor Talon
	 * @param rearCanID The id of the back drive motor Talon, If 0 the class will not use a rear motor controller
	 * @param side Indicates which side of the robot the two drive motors control
	 */
	public DriveUnit(MotorController frontCAN, MotorController rearCAN, UnitSide side) {
    frontMotor = frontCAN;
    rearMotor = rearCAN;
		//If a valid can id has been provided for the rear motor controller
		if (rearCAN != null) {
	
			//Tells the rear motor to do whatever the front motor does
			rearMotor.setFollower(frontMotor);
			
		}
		setBrakeMode(true);
		
		configPID(0.001, 0, 0.1, 1);
		
		unitSide = side;
	}
	
	/**
	 * Sets the speed in Rotations per second
	 * @param rps Rotations per Second
	 */
	public void setVelocityRPS(double rps) {
		if (unitSide == UnitSide.RIGHT) {
			rps = -rps;
		}
		
		frontMotor.setVelocityRPS(rps);
	}
	
	public double getAmps() {
		return frontMotor.getAmps();
	}
	
	/**
	 * Gets the current speed of the motor in RPS
	 * @return Rotations Per Second
	 */
	public double getVelocityRPS() {
		double rps = frontMotor.getVelocityRPS();
		
		if (unitSide == UnitSide.RIGHT) {
			rps = -rps;
		}
		return rps;
	}
	
	/***
	 * Set the speed as a percentage
	 * @param percent any percentage from -1.0 to +1.0
	 */
	public void setPercentSpeed(double percent) {
		if (unitSide == UnitSide.RIGHT){
			percent = -percent;
		}
		frontMotor.setPercentSpeed(percent);
	}
	
	/***
	 * Returns the current voltage the the motors are outputting
	 */
	public double getVoltage(){
		double voltage = frontMotor.getVoltage();
		
		if (unitSide == UnitSide.RIGHT) {
			voltage = -voltage;
		}
		
		return voltage;
	}
	
	/***
	 * Configures the Closed Feed-forward PID loop for the motor controllers
	 * @param p The proportional value of the PID loop
	 * @param i The integral value of the PID loop
	 * @param d The derivative value of the PID loop
	 * @param error The allowable error in the PID loop
	 */
	public void configPID(double p, double i, double d, int error){
		frontMotor.configPID(p,  i,  d,  error);
	}
	
	/**
	 * @return The current error in the PID loop
	 */
	public int getPIDerror() {
		int error = frontMotor.getPIDerror();
		
		if (unitSide == UnitSide.RIGHT) {
			error = -error;
		}
		
		return error;
	}
	
	/**
	 * @return The position, in ticks of the encoder
	 */
	public int getEncoderPosition() {
		int position = frontMotor.getEncoderPosition();
		
		if (unitSide == UnitSide.RIGHT) {
			position = -position;
		}
		return position;
	}
	
	/**
	 * @return The number of Rotations the shaft of the motor has completed
	 */
	public double getRotations() {
		double rotations = frontMotor.getRotations();
		
		if (unitSide == UnitSide.RIGHT) {
			rotations = -rotations;
		}
		return rotations;
	}
	
	/**
	 * @return The velocity of the Encoder in ticks per microsecond
	 */
	public double getRawVelocity() {
		double velocity = frontMotor.getRawVelocity();
		
		if (unitSide == UnitSide.RIGHT) {
			velocity = -velocity;
		}
		return velocity;
	}
	
	/**
	 * @return The velocity in Rotations per minute
	 */
	public double getVelocityRPM() {
		double rpm = getRawVelocity()/rpmToClicksPer100ms;
		
		if (unitSide == UnitSide.RIGHT) {
			rpm = -rpm;
		}
		return rpm;	
	}
	
	public void setVelocityRPM(double rpm) {
		if (unitSide == UnitSide.LEFT) {
			frontMotor.setVelocityRPM(rpm);
		}else {
			frontMotor.setVelocityRPM(-rpm);
		}
	}
	
	/**
	 * Sets the velocity in Feet per second
	 * @param fps The velocity in feet per second
	 */
	public void setVelocityFPS(double fps) {
		//First convert to inches per minute
		double fpm = (fps * 60); //60 sec/min;
		//Convert to rpm
		double rpm = fpm / (wheelDiameter * Math.PI);
		
		setVelocityRPM(rpm);//Now set the speed
	}
	
	
	
	/**
	 * Gets the speed in feet per second
	 * @return Speed in Feet per second
	 */
	public double getVelocityFPS() {
		double currentRpm = getVelocityRPM();
		double fps = (currentRpm * (wheelDiameter * Math.PI))/60;
		
		
		return fps;
	}
	
	/**
	 * Tells the motor to turn to the ticks specified
	 * @param ticks The position in ticks to set the encoder to
	 */
	@SuppressWarnings("unused")
	private void setPositionInTicks(int ticks) {
		if (unitSide == UnitSide.RIGHT) {
			ticks = -ticks;
		}
		frontMotor.setPositionInTicks(ticks);
	}
	
	/**
	 * Tells the motor to rotate the shaft the specified number of times
	 * @param numRotations The number of times to rotate the shaft of the motor
	 */
	public void setRotations(double numRotations) {
		int ticks = (int)rotationsToTicks(numRotations);
		
		if (unitSide == UnitSide.RIGHT) {
			ticks = -ticks;
		}
			
		frontMotor.setPositionInTicks(ticks);
	}
	
	/**
	 * Tells the motor to travel the specified number of feet
	 * @param feet The number of feet to travel
	 */
	public void setFeet(double feet) {
		int ticks = (int)feetToTicks(feet);
		
		if (unitSide == UnitSide.RIGHT) {
			ticks = -ticks;
		}
		frontMotor.setPositionInTicks(ticks);
	}
	
	/**
	 * 
	 * @return Number of feet traveled
	 */
	public double getFeet(){
		double ticks = frontMotor.getEncoderPosition();
		
		if (unitSide == UnitSide.RIGHT) {
			ticks = -ticks;
		}
		return ticksToFeet(ticks);
	}
	
	/**
	 * Converts feet to the number of shaft rotations it would take to travel that far
	 * @param feet 
	 * @return number of shaft rotations to travel the specified number of feet
	 */
	private double feetToRotations(double feet){
		//convert feet to rotation
		return (feet/(wheelDiameter*Math.PI));
	}
	
	/**
	 * Converts the number of rotations to the equivalent distance in feet
	 * @param rotations number of rotations to convert
	 * @return the number of feet the robot would travel with the given number of rotations
	 */
	private double rotationsToFeet(double rotations){
		//convert rotations to feet
		return rotations*wheelDiameter*Math.PI;
	}
	
	/**
	 * Converts rotations to ticks
	 * @param rotations rotations to convert to ticks
	 * @return Number of ticks in specified number of rotations
	 */
	private double rotationsToTicks(double rotations){
		//convert rotations to feet
		return rotations*ticksPerShaftRotation;
	}
	
	/**
	 * Converts ticks to rotations
	 * @param ticks Number of ticks to convert
	 * @return Number of rotations in ticks
	 */
	private double ticksToRotations(double ticks){
		//convert rotations to feet
		return ticks/ticksPerShaftRotation;
	}
	
	/**
	 * Converts feet to equivalent distance traveled in ticks
	 * @param feet Number of feet to convert
	 * @return Number of ticks in feet
	 */
	private double feetToTicks(double feet){
		return rotationsToTicks(feetToRotations(feet));
	}
	
	/**
	 * Converts ticks to equivalent distance in feet
	 * @param ticks Number of ticks to convert
	 * @return Number of feet in ticks
	 */
	private double ticksToFeet(double ticks){
		return rotationsToFeet(ticksToRotations(ticks));
	}
	
	/**
	 * Converts inches to feet as a double so we do not lose any precision when multiplying or dividing 
	 * @param inches 
	 * @return feet
	 */
	public double inchesToFeet(double inches) {
		return inches/12.0;
	}
	
	/**
	 * Resets the encoder position to 0
	 */
	public void zeroEncoder() {
		frontMotor.zeroEncoder();
	}
	
	/**
	 * Turn on or off the brakes
	 * @param brake Brake or Coast
	 */
	public void setBrakeMode(Boolean brake){
    frontMotor.setBrakeMode(brake);
    if (rearMotor != null) {
      rearMotor.setBrakeMode(brake);
    }
	}
}


	