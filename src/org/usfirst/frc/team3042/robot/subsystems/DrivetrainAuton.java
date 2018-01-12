package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Logger;
import org.usfirst.frc.team3042.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;


/** DrivetrainAuton ***********************************************************
 * The methods and information necessary for autonomous motion profile driving.
 */
public class DrivetrainAuton extends Subsystem {
	/** Configuration Constants ***********************************************/
	private static final Logger.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN_AUTON;
	private static final int PID_PROFILE = RobotMap.AUTON_PROFILE;
	private static final double kP = RobotMap.kP_AUTON;
	private static final double kI = RobotMap.kI_AUTON;
	private static final double kD = RobotMap.kD_AUTON;
	private static final double kF_LEFT = RobotMap.kF_DRIVE_LEFT;
	private static final double kF_RIGHT = RobotMap.kF_DRIVE_RIGHT;
	private static final int I_ZONE = RobotMap.I_ZONE_AUTON;
	//The Frame Rate is given in ms
	private static final int FRAME_RATE = RobotMap.AUTON_FRAME_RATE;

	
	/** Periodic Runnable *****************************************************
	 * Create a separate thread to push motion profile points out to the Talon
	 */
	class PeriodicRunnable implements java.lang.Runnable {
		public void run() { 
			leftMotor.processMotionProfileBuffer();
			rightMotor.processMotionProfileBuffer();
		}
	}
    	
	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, getName());
	TalonSRX leftMotor, rightMotor;
	DrivetrainEncoders encoders;
	Notifier notifier;
	

	/** DrivetrainAuton *******************************************************/
	public DrivetrainAuton(TalonSRX leftMotor, TalonSRX rightMotor, 
			DrivetrainEncoders encoders) {
		log.add("Constructor", LOG_LEVEL);
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.encoders = encoders;
		
		initMotor(leftMotor, kF_LEFT);
		initMotor(rightMotor, kF_RIGHT);;

		/** Starting talons processing motion profile **/
		//Convert from ms to sec for the notifier
		double frameRateSec = (double)FRAME_RATE / 1000.0;
		notifier = new Notifier(new PeriodicRunnable());
		notifier.startPeriodic(frameRateSec);
	}
	private void initMotor(TalonSRX motor, double kF) {
		motor.changeMotionControlFramePeriod(FRAME_RATE);
		motor.setProfile(PID_PROFILE);
		motor.setPID(kP, kI, kD);
		motor.setF(kF);
		motor.setIZone(I_ZONE);
	}
	
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem.
	 */
	public void initDefaultCommand() {
	}
	

	/** prepareMotionProfile *****************************************************
	 * Clears out any old trajectories and prepares to receive new trajectory 
	 * points.
	 */
	public void initMotionProfile() {		
		initMotor(leftMotor);
		initMotor(rightMotor);
		disableMotionProfile();
		removeUnderrun();
		encoders.setToZero();
	}
	private void initMotor(TalonSRX motor) {
		motor.clearMotionProfileTrajectories();
		motor.setProfile(PID_PROFILE);
		motor.changeControlMode(TalonControlMode.MotionProfile);
	}
	
	
	/** Motion Profile command methods ****************************************/
	public void pushPoints(	TalonSRX.TrajectoryPoint leftPoint, 
							TalonSRX.TrajectoryPoint rightPoint) {
		leftMotor.pushMotionProfileTrajectory(leftPoint);
		rightMotor.pushMotionProfileTrajectory(rightPoint);
	}
	public MotionProfileStatus getLeftStatus() {
		MotionProfileStatus status = new MotionProfileStatus();
		leftMotor.getMotionProfileStatus(status);
		return status;
	}
	public MotionProfileStatus getRightStatus() {
		MotionProfileStatus status = new MotionProfileStatus();
		rightMotor.getMotionProfileStatus(status);
		return status;
	}
	public void removeLeftUnderrun() {
		leftMotor.clearMotionProfileHasUnderrun();
	}
	public void removeRightUnderrun() {
		rightMotor.clearMotionProfileHasUnderrun();
	}
	private void removeUnderrun() {
		removeLeftUnderrun();
		removeRightUnderrun();
	}
	public void enableMotionProfile() {
		leftMotor.set(TalonSRX.SetValueMotionProfile.Enable.value);
		rightMotor.set(TalonSRX.SetValueMotionProfile.Enable.value);
	}
	public void holdMotionProfile() {
		leftMotor.set(TalonSRX.SetValueMotionProfile.Hold.value);
		rightMotor.set(TalonSRX.SetValueMotionProfile.Hold.value);
	}
	public void disableMotionProfile() {
		leftMotor.set(TalonSRX.SetValueMotionProfile.Disable.value);
		rightMotor.set(TalonSRX.SetValueMotionProfile.Disable.value);
	}
}