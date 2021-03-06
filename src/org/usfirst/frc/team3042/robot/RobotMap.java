package org.usfirst.frc.team3042.robot;

import org.usfirst.frc.team3042.lib.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	/** Robot selector ********************************************************/
	public static enum Bot {PBOT, PRIMARY, SECONDARY;}
	// Set the bot to which you intend to push code.
	private static Bot currentBot = Bot.PBOT;

	public static final boolean IS_PBOT 	= (currentBot == Bot.PBOT);
	public static final boolean IS_PRIMARY = (currentBot == Bot.PRIMARY);
	public static final boolean IS_SECONDARY = (currentBot == Bot.SECONDARY);
	
	
	/** Robot Size Parameters *************************************************
	 * The units of the wheel diameter determine the units of the position 
	 * and speed closed-loop commands. For example, if the diameter is given 
	 * in inches, position will be in inches and speed in inches per second.
	 */
	public static final double WHEEL_DIAMETER = 4.0;
	public static final double ROBOT_WIDTH = (IS_PBOT) ? 20.2 : 0.0;
	
	
	/** USB ports *************************************************************/					
	public static final int USB_JOYSTICK_LEFT 	= 0;
	public static final int USB_JOYSTICK_RIGHT 	= 1;
	public static final int USB_GAMEPAD 		= IS_PBOT ? 0 : 2;


	/** PWM ports *************************************************************/
	public static final int PWM_PAN_PORT 	= 0;
	public static final int PWM_TILT_PORT 	= 1;
	
	
	/** CAN ID numbers ********************************************************/
	public static final int CAN_LEFT_MOTOR 	= 		IS_PBOT 	? 3 :
													IS_PRIMARY 	? 0 : 0;
	public static final int CAN_RIGHT_MOTOR = 		IS_PBOT 	? 9 :
													IS_PRIMARY 	? 0 : 0;
	public static final int CAN_LEFT_FOLLOWER = 	IS_PRIMARY 	? 0 : 0;
	public static final int CAN_RIGHT_FOLLOWER = 	IS_PRIMARY 	? 0 : 0;
	public static final int CAN_SPINNER 	= 		IS_PBOT		? 10 :
													IS_PRIMARY 	? 0 : 0;
	
	
	/** PCM channels **********************************************************/
	public static final int LIGHT_RING_CHANNEL = 1;
	
	
	/** OI Settings ***********************************************************/
	public static final boolean USE_JOYSTICKS = !IS_PBOT;
	public static final double JOYSTICK_DRIVE_SCALE = 0.5;
	public static final double TRIGGER_SPINNER_SCALE = 0.1;
	public static final double JOYSTICK_DEAD_ZONE = 0.0;


	/** Drivetrain Settings ***************************************************/
	public static final boolean HAS_DRIVETRAIN = true;
	public static final boolean HAS_FOLLOWERS = !IS_PBOT;
	public static final NeutralMode DRIVETRAIN_BRAKE_MODE = NeutralMode.Brake;
	public static final boolean REVERSE_LEFT_MOTOR = 	(IS_PBOT) ? true : false;
	public static final boolean REVERSE_RIGHT_MOTOR = 	(IS_PBOT) ? false: false;
	// Maximum Acceleration given in power per second
	public static final double ACCELERATION_MAX = 1.5;
	public static final double kF_DRIVE_LEFT = 	(IS_PBOT) 		? 1.02 :
												(IS_PRIMARY) 	? 0.0 : 0.0;
	public static final double kF_DRIVE_RIGHT = (IS_PBOT) 		? 0.774 :
												(IS_PRIMARY) 	? 0.0 : 0.0;
	public static final int TALON_ERROR_TIMEOUT = 0;// measured in Ms
	public static final int TRAJPERIOD = 10;
	public static final int PIDIDX = 0; //pidIdx - 0 for Primary closed-loop. 1 for cascaded closed-loop. See Phoenix-Documentation for how to interpret.
	public static final int SLOTIDX_1 = 0;
	
	
	/** Drivetrain Encoder Settings *******************************************/
	public static final boolean HAS_ENCODERS = true;
	//Encoder counts per revolution
	//In quadrature mode, actual counts will be 4x this; e.g., 360 -> 1440
	public static final int COUNTS_PER_REVOLUTION = 360;
	//How often the encoders update on the CAN, in milliseconds
	public static final int ENCODER_FRAME_RATE = 10;
	public static final boolean REVERSE_LEFT_ENCODER = true;
	public static final boolean REVERSE_RIGHT_ENCODER = false;
	
	
	/** Drivetrain Autonomous Settings ****************************************/
	public static final boolean HAS_AUTON = HAS_ENCODERS;
	public static final int AUTON_PROFILE = 0;
	public static final double kP_AUTON = 		(IS_PBOT) 		? 0.4 :
												(IS_PRIMARY) 	? 0.0 : 0.0;
	public static final double kI_AUTON = 		(IS_PBOT) 		? 0.0 :
												(IS_PRIMARY) 	? 0.0 : 0.0;
	public static final double kD_AUTON = 		(IS_PBOT) 		? 0.8 :
												(IS_PRIMARY) 	? 0.0 : 0.0;
	public static final int I_ZONE_AUTON =		(IS_PBOT)		? 0 :
												(IS_PRIMARY)	? 0 : 0;
	//The rate of pushing motion profile points to the talon, in ms
	public static final int AUTON_FRAME_RATE = 5;
	//Parameters for calibrating the F-gain
	public static final double AUTON_CALIBRATE_POWER = 0.2;
	public static final double AUTON_CALIBRATE_TIME = 5.0; //seconds
	public static final int AUTON_COUNT_AVERAGE = 20;
	//Parameters for motion profile driving
	public static final double AUTON_DT = 0.01; //time interval in sec
	public static final double AUTON_ACCEL_TIME = 0.5; //time in sec
	public static final double AUTON_SMOOTH_TIME = 0.1; //time in sec
	public static final double AUTON_MAX_ACCEL = 3.0; //rev per sec per sec
	public static final int AUTON_BUFFER_TRIGGER = 5;
	
	
	/** Drivetrain Gyro Drive Settings ****************************************/
	public static final double kP_GYRO = 0.055;
	public static final double kI_GYRO = 0.0;
	public static final double kD_GYRO = 0.11;
	public static final double ANGLE_TOLERANCE = 2.0;
	public static final double MAX_SPEED_GYRO = 0.4;
	
	
	/** Spinner Settings ******************************************************/
	public static final boolean HAS_SPINNER = IS_PBOT;
	public static final boolean SPINNER_BRAKE_MODE = true;
	public static final boolean REVERSE_SPINNER = false;
	
	
	/** Spinner Encoder Settings **********************************************/
	public static final boolean HAS_SPINNER_ENCODER = HAS_SPINNER;
	public static final int SPINNER_ENCODER_FRAME_RATE = 10;
	public static final int SPINNER_ENCODER_COUNTS_PER_REV = 1024;
	public static final boolean REVERSE_SPINNER_ENCODER = false;
	
	
	/** Spinner Closed-Loop Settings ******************************************/
	public static boolean HAS_SPINNER_CLOSED_LOOP = HAS_SPINNER;
	public static int SPINNER_POSITION_PROFILE = 0;
	public static final double kP_SPINNER_POSITION = 0.51;
	public static final double kI_SPINNER_POSITION = 0.0;
	public static final double kD_SPINNER_POSITION = 5.1;
	public static int SPINNER_SPEED_PROFILE = 1;
	public static final double kP_SPINNER_SPEED = 0.02;
	public static final double kI_SPINNER_SPEED = 0.0;
	public static final double kD_SPINNER_SPEED = 0.2;
	public static final double kF_SPINNER_SPEED = 0.0245;
	public static final double SPINNER_DEFAULT_POSITION = 1.0; //revolutions
	public static final double SPINNER_DEFAULT_SPEED = 500; //RPM
	public static final double SPINNER_CALIBRATE_POWER = 0.1;
	public static final double SPINNER_CALIBRATE_TIME = 10.0; //seconds
	public static final int SPINNER_COUNT_AVERAGE = 20;
	
	
	/** PanTilt Settings ******************************************************/
	public static final boolean HAS_PAN_TILT = IS_PBOT;
	//PWM bounds are for the HS-5685MH servo
	public static final double SERVO_PWM_MAX = 2.25;
	public static final double SERVO_PWM_MIN = 0.76;
	public static final double PAN_MIN = 0.25;
	public static final double PAN_CENTER = 0.430;
	public static final double PAN_MAX = 0.7;
	public static final double TILT_MIN = 0.0;
	public static final double TILT_CENTER 	= 0.515;
	public static final double TILT_MAX = 0.7;
	//The change in servo position per second when driven with the POV buttons
	public static final double SERVO_SPEED = 0.25;
	//Reverse the direction of the servos if they don't match the controls
	public static final boolean REVERSE_PAN = false;
	public static final boolean REVERSE_TILT = false;
	
	
	/** Gyroscope Settings ****************************************************/
	public static final boolean HAS_GYROSCOPE = true;
	public static final double GYROSCOPE_SCALE = 1;
	
	
	/** LEDRing Settings ******************************************************/
	public static final boolean HAS_LIGHT_RING = true;

	
	/** Logger Settings *******************************************************/
	public static final String 		LOG_FILE_FORMAT = "yyyy-MM-dd-hhmmss";
	public static final String 		LOG_TIME_FORMAT = "hh:mm:ss:SSS";
	public static final String 		LOG_DIRECTORY_PATH = "/home/lvuser/logs/";
	public static final String 		LOG_TIME_ZONE = "America/Chicago";
	public static final boolean 	LOG_TO_CONSOLE 				= true;
	public static final boolean 	LOG_TO_FILE 				= false;
	public static final Logger.Level 	LOG_GLOBAL 					= Logger.Level.DEBUG;
	public static final Logger.Level 	LOG_ROBOT 					= Logger.Level.TRACE;
	public static final Logger.Level	LOG_OI 						= Logger.Level.TRACE;
	public static final Logger.Level	LOG_AXIS_TRIGGER 			= Logger.Level.ERROR;
	public static final Logger.Level	LOG_POV_BUTTON				= Logger.Level.ERROR;
	/** Subsystems **/                                                   
	public static final Logger.Level	LOG_DRIVETRAIN				= Logger.Level.TRACE;
	public static final Logger.Level	LOG_DRIVETRAIN_FOLLOWERS	= Logger.Level.TRACE;
	public static final Logger.Level	LOG_DRIVETRAIN_ENCODERS 	= Logger.Level.DEBUG;
	public static final Logger.Level	LOG_DRIVETRAIN_AUTON		= Logger.Level.DEBUG;
	public static final Logger.Level	LOG_SPINNER					= Logger.Level.TRACE;
	public static final Logger.Level	LOG_SPINNER_ENCODER			= Logger.Level.TRACE;
	public static final Logger.Level	LOG_SPINNER_CLOSED_LOOP		= Logger.Level.DEBUG;
	public static final Logger.Level 	LOG_PAN_TILT 				= Logger.Level.TRACE;
	public static final Logger.Level	LOG_GYROSCOPE				= Logger.Level.DEBUG;
	public static final Logger.Level	LOG_LIGHT_RING				= Logger.Level.TRACE;
}