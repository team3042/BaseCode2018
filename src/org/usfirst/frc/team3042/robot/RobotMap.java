package org.usfirst.frc.team3042.robot;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // This is where we basically put all of the robot configuration information
	
	// Choose a robot
	public static enum Robot{PRIMARY, SECONDARY, PROGRAMMING};
	
	private static Robot currentRobot = Robot.PRIMARY;
	
	private static final boolean IS_PRIMARY = currentRobot == Robot.PRIMARY;
	private static final boolean IS_SECONDARY = currentRobot == Robot.SECONDARY;
	private static final boolean IS_PROGRAMMING = currentRobot == Robot.PROGRAMMING;
	
	
	// Robot dimensions
	public static final double DRIVE_WHEEL_DIAMETER = 4.0;
	public static final double WHEEL_BASE_WIDTH = (IS_PROGRAMMING)? 20.2 : 0.0 /* Measure the robot! */;
	
	// USB ports
	public static final int JOYSTICK_LEFT = 0;
	public static final int JOYSTICK_RIGHT = 1;
	public static final int GAMEPAD = 2;
	
	// PWM ports
	public static final int PAN_SERVO = 0;
	public static final int TILT_SERVO = 1;
	
	// CAN IDs
	public static final int LEFT_MOTOR = (IS_PRIMARY)? 0 : (IS_SECONDARY)? 0 : 0;
}
