package org.usfirst.frc.team3042.lib;

import org.usfirst.frc.team3042.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motion.TrajectoryPoint;

/** MotionProfile *************************************************************
 * All distances are assumed to be in revolutions.
 * All Times are assumed to be in seconds.
 */
public class MotionProfile {
	/** Configuration Constants ***********************************************/
	private static final Logger.Level LOG_LEVEL = RobotMap.LOG_DRIVETRAIN_AUTON;
	private static final int PROFILE = RobotMap.AUTON_PROFILE;
	private static final double DT = RobotMap.AUTON_DT;
	private static final double ACCEL_TIME = RobotMap.AUTON_ACCEL_TIME;
	private static final double SMOOTH_TIME = RobotMap.AUTON_SMOOTH_TIME;
	private static final double MAX_ACCEL = RobotMap.AUTON_MAX_ACCEL;
	
	
	/** Instance Variables ****************************************************/
	Logger log = new Logger(LOG_LEVEL, "MotionProfile");
	TrajectoryPoint[] trajectory;

	
	/**MotionProfile **********************************************************
	 * distance given in rotations
	 * Speed given in rps
	 */
	public MotionProfile(double[] distance, double[] speed) {
		checkAccel(speed);
		
		double[] velocity = calculateVelocity(distance, speed);
		double[] position = calculatePosition(velocity);

		fillTrajectory(position, velocity);
	}
	
	
	/** checkAccel ************************************************************/
	private void checkAccel(double[] speed) {
		int stages = speed.length;

		checkAccel(0.0, speed[0]);
		for (int stage=1; stage<stages; stage++) {
			checkAccel(speed[stage-1], speed[stage]);
		}
		checkAccel(speed[stages-1], 0.0);
	}
	private void checkAccel(double speed0, double speed1) {
		double deltaSpeed = Math.abs(speed1 - speed0);
		double accel = deltaSpeed / ACCEL_TIME;
		if (accel > MAX_ACCEL) {
			log.add("Warning: Maximum Acceleration Exceeded from "
					+speed0+" to "+speed1, Logger.Level.WARNING);
		}
	}
	
	
	/** calculateVelocity *****************************************************/
	private double[] calculateVelocity(double[] distance, double[] speed) {
		int stages = speed.length;

		/** Calculate array length ************************/
		int accelPoints = (int)Math.floor(ACCEL_TIME/DT);
		int smoothPoints = (int)Math.floor(SMOOTH_TIME/DT);
		int length = accelPoints + smoothPoints;
		int[] flatPoints = new int[stages];
		for (int stage=0; stage<stages; stage++) {
			double flatTime = Math.abs(distance[stage]/speed[stage]);
			flatPoints[stage] = (int)Math.floor(flatTime/DT);
			length += flatPoints[stage];
		}

		/** Generate Flat velocity array ******************/
		double[] velocity = new double[length];
		int n=0;
		velocity[n] = 0.0;
		for (int stage=0; stage<stages; stage++) {
			for (int i=0; i<flatPoints[stage]; i++) {
				n++;
				velocity[n] = speed[stage];
			}
		}
		for (int i=1; i<(accelPoints+smoothPoints); i++) {
			n++;
			velocity[n] = 0.0;
		}
		
		/** Smooth the velocity profile *******************/
		velocity = boxcarSmooth(velocity, accelPoints);
		velocity = boxcarSmooth(velocity, smoothPoints);
		
		return velocity;
	}
	
	/** boxcarSmooth **********************************************************/
	private double[] boxcarSmooth(double[] unsmooth, int box) {
		int length = unsmooth.length;
		double[] smooth = new double[length];

		for (int n=0; n<length; n++) {
			int i0 = Math.max(0,  1+n-box);
			for (int i=i0; i<=n; i++) {
				smooth[n] += unsmooth[i];
			}
			smooth[n] /= box;
		}
		return smooth;
	}
	
	
	/** calculatePosition *****************************************************/
	private double[] calculatePosition(double[] velocity) {
		int length = velocity.length;
		double[] position = new double[length];
		
		position[0] = 0.0;
		for (int n=1; n<length; n++) {
			double averageVelocity = 0.5 * (velocity[n-1] + velocity[n]);
			position[n] = position[n-1] + averageVelocity * DT;
		}
		return position;
	}
	
	
	/** fillTrajectory ********************************************************/
	private void fillTrajectory(double[] position, double[] velocity) {
		int length = velocity.length;
		trajectory = new TrajectoryPoint[length];
		
		for (int n=0; n<length; n++) {
			trajectory[n] = new CANTalon.TrajectoryPoint();
			trajectory[n].timeDurMs = (int)(DT * 1000.0); //convert to ms for Talon
			trajectory[n].position = position[n];
			trajectory[n].velocity = velocity[n] * 60.0; //convert to rpm for Talon
			trajectory[n].profileSlotSelect = PROFILE;
			trajectory[n].velocityOnly = false;
			trajectory[n].zeroPos = (n == 0);
			trajectory[n].isLastPoint = (n == length-1);
		}
	}
	
	/** getPoint **************************************************************/
	public TalonSRX.TrajectoryPoint getPoint(int n) {
		TalonSRX.TrajectoryPoint point = null;
		if ( (n>= 0) && (n<getLength()) ) {
			point = trajectory[n];
		}
		else {
			log.add("Array out of bounds: getPoint()", Log.Level.ERROR);
		}
		return point;
	}
	
	
	/** getLength *************************************************************/
	public int getLength() {
		return trajectory.length;
	}
}