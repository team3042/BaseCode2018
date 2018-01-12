package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.Logger;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.DrivetrainEncoders_Dashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/** DrivetrainEncoders ***********************************************************
 * The encoders for the drivetrain.
 */
public class DrivetrainEncoders extends Subsystem {
	/** Configuration Constants ***********************************************/
	private static final Logger.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN_ENCODERS;
	private static final int COUNTS_PER_REVOLUTION = RobotMap.COUNTS_PER_REVOLUTION;
	private static final int FRAME_RATE = RobotMap.ENCODER_FRAME_RATE;
	private static final boolean REVERSE_LEFT = RobotMap.REVERSE_LEFT_ENCODER;
	private static final boolean REVERSE_RIGHT = RobotMap.REVERSE_RIGHT_ENCODER;

	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, getName());
	TalonSRX leftEncoder, rightEncoder;
	double leftPositionZero, rightPositionZero;
	int leftCountsZero, rightCountsZero;
	
	
	/** DrivetrainEncoders ****************************************************/
	public DrivetrainEncoders(TalonSRX leftMotor, TalonSRX rightMotor) {
		log.add("Constructor", LOG_LEVEL);
		
		leftEncoder = leftMotor;
		rightEncoder = rightMotor;
				
		initEncoder(leftEncoder, REVERSE_LEFT);
		initEncoder(rightEncoder, REVERSE_RIGHT);
													
		reset();
	}
	private void initEncoder(TalonSRX encoder, boolean reverse) {
		encoder.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		encoder.setStatusFrameRateMs(TalonSRX.StatusFrameRate.QuadEncoder, FRAME_RATE);
		encoder.configEncoderCodesPerRev(COUNTS_PER_REVOLUTION);
		encoder.reverseSensor(reverse);
	}
	
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new DrivetrainEncoders_Dashboard());
	}

	
	/** reset *****************************************************************/
	public void reset() {
		leftPositionZero = leftEncoder.getPosition();
		rightPositionZero = rightEncoder.getPosition();
				
		leftCountsZero = leftEncoder.getEncPosition();
		rightCountsZero = rightEncoder.getEncPosition();
	}
	public void setToZero() {
		leftPositionZero = 0.0;
		rightPositionZero = 0.0;
		
		leftCountsZero = 0;
		rightCountsZero = 0;
	}
	
	
	/** Get the encoder position or speed *************************************
	 * Position is given in Revolutions - getPosition()
	 * Counts given in encoder counts - getEncPosition()
	 * Speed returns RPM - getSpeed()
	 * Encoder speed returns counts per 100ms - getEncSpeed()
	 */
	public double getLeftPosition() {
		return leftEncoder.getPosition() - leftPositionZero;
	}
	public double getRightPosition() {
		return rightEncoder.getPosition() - rightPositionZero;
	}
	public int getLeftCounts() {
		return leftEncoder.getEncPosition() - leftCountsZero;
	}
	public int getRightCounts() {
		return rightEncoder.getEncPosition() - leftCountsZero;
	}
	public double getLeftSpeed() {
		return leftEncoder.getSpeed();
	}
	public double getRightSpeed() {
		return rightEncoder.getSpeed();
	}
	
	
	/** rpmToF ****************************************************************
	 * Convert RPM reading into an F-Gain
	 * Note that 1023 is the native full-forward power of the talons, 
	 * equivalent to setting the power to 1.0.
	 * The speed has to be converted from rpm to encoder counts per 100ms
	 * 
	 * so F = power * 1023 / speed
	 */
	public double rpmToF(double rpm, double power) {
		//Convert to counts per 100 ms
		double speed = rpm * 4 * COUNTS_PER_REVOLUTION / 600;
		double kF = power * 1023.0 / speed;
		return kF;
	}
	public double rpmToPower(double rpm, double kF) {
		//Convert to counts per 100 ms
		double speed = rpm * 4 * COUNTS_PER_REVOLUTION / 600;
		double power = kF * speed / 1023.0;
		return power;
	}
}