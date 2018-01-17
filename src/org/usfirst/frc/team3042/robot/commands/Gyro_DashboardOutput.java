package org.usfirst.frc.team3042.robot.commands;

import org.usfirst.frc.team3042.lib.Logger;
import org.usfirst.frc.team3042.robot.Robot;
import org.usfirst.frc.team3042.robot.RobotMap;
import org.usfirst.frc.team3042.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Gyro_DashboardOutput extends Command {
	
	/** Configuration Constants ***********************************************/
	private static final Logger.Level LOG_LEVEL = RobotMap.LOG_GYROSCOPE;
	
	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, getName());
	Gyro gyroscope = Robot.gyro;
	
	
	/** Gyroscope_Dashboard ***************************************************
	 * Required subsystems will cancel commands when this command is run.
	 */
    public Gyro_DashboardOutput() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	log.add("Constructor", Logger.Level.TRACE);
		
		requires(gyroscope);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	log.add("Initialize", Logger.Level.TRACE);
		gyroscope.reset();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Gyroscope (Degrees)", gyroscope.getAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	log.add("End", Logger.Level.TRACE);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	log.add("Interrupted", Logger.Level.TRACE);
    }
}
