package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Logger;
import org.usfirst.frc.team3042.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/** DrivetrainFollowers **********************************************************
 * Motor controllers for secondary drivetrain motors
 */
public class DrivetrainFollowers extends Subsystem {
	/** Configuration Constants ***********************************************/
	public static final Logger.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN_FOLLOWERS;
	private static final int CAN_LEFT_MOTOR = RobotMap.CAN_LEFT_MOTOR;
	private static final int CAN_RIGHT_MOTOR = RobotMap.CAN_RIGHT_MOTOR;
	private static final int CAN_LEFT_FOLLOWER = RobotMap.CAN_LEFT_FOLLOWER;
	private static final int CAN_RIGHT_FOLLOWER = RobotMap.CAN_RIGHT_FOLLOWER;

	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, getName());
	TalonSRX leftFollower = new TalonSRX(CAN_LEFT_FOLLOWER);
	TalonSRX rightFollower = new TalonSRX(CAN_RIGHT_FOLLOWER);	
	

	/** DrivetrainFollowers **************************************************/
	public DrivetrainFollowers() {
		log.add("Constructor", Logger.Level.TRACE);
		
		initMotor(leftFollower, CAN_LEFT_MOTOR);
		initMotor(rightFollower, CAN_RIGHT_MOTOR);
	}
	private void initMotor(TalonSRX motor, int leaderCANid) {
		motor.changeControlMode(TalonControlMode.Follower);
		motor.set(leaderCANid);
	}
	
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem.
	 */
	public void initDefaultCommand() {
	}
}