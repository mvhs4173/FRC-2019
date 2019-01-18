package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorController {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private TalonSRX controller;
	private final int standardTimeoutMs = 10;
	private int ticksPerShaftRotation = 4096;
	private final double rpmToClicksPer100ms =  ticksPerShaftRotation/1.0 * 1.0/60.0 * 1.0/10.0;// rev/min = 4096 clicks/rev * 1min/60s * 1s/10 centi seconds
	private final double rpsToClicksPer100ms = rpmToClicksPer100ms*60;
	private final double actualOverRequestedRPM =  0.85589;
	private int origin = 0;
	
	/***
	 * 
	 * @param The id of the motor controller
	 */
	public MotorController(int controllerId) {
		controller = new TalonSRX(controllerId);
		
		//Tell the motors to brake if there is no voltage being applied to them
		controller.setNeutralMode(NeutralMode.Brake);
		configPID(0.01, 0.0, 0.1, 1);
		//Set the thing that we use as an encoder
		ticksPerShaftRotation = 4096;
	}
	
	
	public void enableForwardLimitSwitch() {
		//Tell the controller where the limit switch is plugged in
		controller.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, standardTimeoutMs);
		controller.overrideLimitSwitchesEnable(false);//Enable switch
	}
	
	public void enableReverseLimitSwitch() {
		//Tell the controller where the limit switch is plugged in
		controller.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,  LimitSwitchNormal.NormallyOpen, standardTimeoutMs);
		controller.overrideLimitSwitchesEnable(false);//Enable switch
	}
	
	/**
	 * Set how long it should take for the motors to accelerate to the desired speed
	 * @param secondsToAccelerate How many seconds it should take to accelerate
	 */
	public void setMaxAccelerationTime(double secondsToAccelerate) {
		controller.configClosedloopRamp(secondsToAccelerate,  10);
	}
	
	/**
	 * Sets the feedback device to the CTRE mag encoder
	 */
	public void configQuadEncoder() {
		controller.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,  0,  standardTimeoutMs);
		ticksPerShaftRotation = 1024;
	}
	
	/**
	 * Resets the encoder so that its current position is considered 0
	 */
	public void resetEncoder() {
		controller.setSelectedSensorPosition(0,  0,  0);
	}
	
	/**
	 * Sets the feedback device to the CTRE mag encoder
	 */
	public void configMagEncoder() {
		controller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, standardTimeoutMs);
		ticksPerShaftRotation = 4096;
	}
	
	/**
	 * Gets the Id of the motor controller
	 * @return The id of the motor controller
	 */
	public int getControllerId() {
		return controller.getDeviceID();
	}
	
	/**
	 * Sets which motor controller to follow
	 * @param leader The motor controller to follow
	 */
	public void setFollower(MotorController leader) {
		controller.set(ControlMode.Follower, leader.getControllerId());
	}
	
	/***
	 * Set the speed as a percentage
	 * @param percent any percentage from -1.0 to +1.0
	 */
	public void setPercentSpeed(double percent) {
		controller.set(ControlMode.PercentOutput ,percent);
	}
	
	/***
	 * Returns the current voltage the the motors are outputting
	 */
	public double getVoltage(){
		return controller.getMotorOutputVoltage();
	}
	
	/***
	 * Configures the Closed Feed-forward PID loop for the motor controllers
	 * @param p The proportional value of the PID loop
	 * @param i The integral value of the PID loop
	 * @param d The derivative value of the PID loop
	 * @param error The allowable error in the PID loop
	 */
	public void configPID(double p, double i, double d, int error){
		controller.config_kF(0, 0.23, standardTimeoutMs);
		controller.config_kP(0, p, standardTimeoutMs);
		controller.config_kI(0, i, standardTimeoutMs);
		controller.config_kD(0, d, standardTimeoutMs);
		controller.configAllowableClosedloopError(0, error, standardTimeoutMs);
	}
	
	/**
	 * @return The current error in the PID loop
	 */
	public int getPIDerror() {
		return controller.getClosedLoopError(0);
	}
	
	/**
	 * @return The position, in ticks of the encoder
	 */
	public int getEncoderPosition() {
		return controller.getSelectedSensorPosition(0) - origin;
	}
	
	/**
	 * @return The number of Rotations the shaft of the motor has completed
	 */
	public double getRotations() {
		return controller.getSelectedSensorPosition(0)/ticksPerShaftRotation;
	}
	
	/**
	 * @return The velocity of the Encoder in ticks per microsecond
	 */
	public double getRawVelocity() {
			return controller.getSelectedSensorVelocity(0);
	}
	
	/**
	 * @return The velocity in Rotations per minute
	 */
	public double getVelocityRPM() {
		return getRawVelocity()/rpmToClicksPer100ms;	
	}
	
	/**
	 * The velocity in Rotations per second
	 * @return Rotations per second
	 */
	public double getVelocityRPS() {
		return getVelocityRPM() / 60;
	}
	
	/**
	 * Sets the speed of the motor to the speed given
	 * @param rps Rotations per second
	 */
	public void setVelocityRPS(double rps) {
		double clicksPer100ms = rps * rpsToClicksPer100ms;
		
		double finalSpeed = clicksPer100ms / actualOverRequestedRPM;// + 231;
		
		controller.set(ControlMode.Velocity, finalSpeed);
		
		SmartDashboard.putNumber("Requested RPS",  rps);
		SmartDashboard.putNumber("Requested clicks per 100 ms",  clicksPer100ms);
	}
	
	/**
	 * Sets the speed of the motor to the speed given
	 * @param rpm Rotations per minute
	 */
	public void setVelocityRPM(double rpm) {
		double clicksPer100ms = rpm * rpmToClicksPer100ms;
		
		double finalSpeed = clicksPer100ms / actualOverRequestedRPM;// + 231;
		
		controller.set(ControlMode.Velocity, finalSpeed);
		
		SmartDashboard.putNumber("Requested RPM",  rpm);
		SmartDashboard.putNumber("Requested clicks per 100 ms",  clicksPer100ms);
	}
	
	/**
	 * Tells the motor to turn to the ticks specified
	 * @param origin2 The position in ticks to set the encoder to
	 */
	public void setPositionInTicks(double origin2) {
			controller.set(ControlMode.Position, origin2);
	}
	
	/**
	 * Tells the motor to rotate the shaft the specified number of times
	 * @param numRotations The number of times to rotate the shaft of the motor
	 */
	public void setRotations(double numRotations) {
			controller.set(ControlMode.Position, rotationsToTicks(numRotations));
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
	@SuppressWarnings("unused")
	private double ticksToRotations(double ticks){
		//convert rotations to feet
		return ticks/ticksPerShaftRotation;
	}
	
	/**
	 * Resets the encoder position to 0
	 */
	public void zeroEncoder() {
		//controller.setSelectedSensorPosition(0, 0, standardTimeoutMs);
		origin = controller.getSelectedSensorPosition(0);
	}
	
	/**
	 * Turn on or off the brakes
	 * @param brake Brake or Coast
	 */
	public void setBrake(NeutralMode brake){
		controller.setNeutralMode(brake);
	}
	
	public double getAmps() {
		return controller.getOutputCurrent();
	}
}	