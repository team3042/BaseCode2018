package org.usfirst.frc.team3042.robot.subsystems;

import org.usfirst.frc.team3042.lib.ADIS16448_IMU;
import org.usfirst.frc.team3042.lib.Logger;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.commands.Gyro_DashboardOutput;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Gyro extends Subsystem {

	/** Configuration Constants ***********************************************/
	private static final Logger.Level LOG_LEVEL = RobotMap.LOG_GYROSCOPE;
	private static final double GYROSCOPE_SCALE = RobotMap.GYROSCOPE_SCALE;

	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, getName());
    ADIS16448_IMU gyroscope = new ADIS16448_IMU();

	
	/** ExampleSubsystem ******************************************************/
	public Gyro() {
		log.add("Constructor", LOG_LEVEL);
		reset();
		calibrate();
	}
	
	
	/** initDefaultCommand ****************************************************
	 * Set the default command for the subsystem.
	 */
	public void initDefaultCommand() {
		setDefaultCommand(new Gyro_DashboardOutput());
	}
	
	
	/** Command Methods *******************************************************/
	public double getAngle() {
		return gyroscope.getAngleZ()*GYROSCOPE_SCALE;	
	}
	public void reset() {
		gyroscope.reset();
	}
	public void calibrate() {
		gyroscope.calibrate();
	}
}

