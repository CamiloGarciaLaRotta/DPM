package utilities;
import chassis.Main;
/* 
 *
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Helpful tools to control robot navigation
 * @version 0.1
 * @author juliette
 *
 */
public class Navigation {
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	public static boolean PathBlocked = false;

	/**
	 * Navigation Constructor
	 * @param odometer - Odometer object
	 */
	public Navigation(Odometer odometer) {
		this.odometer = odometer;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * Function to set the motor speeds jointly
	 * @param lSpd - speed of left motor
	 * @param rSpd - speed of right motor
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * Function to set the motor speeds jointly
	 * @param lSpd - speed of left motor
	 * @param rSpd - speed of right motor
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/**
	 * TravelTo function that travels to designated position while constantly updating heading
	 * @param x - x position
	 * @param y - y position
	 */
	public void travelTo(double x, double y) {
		Navigation.PathBlocked = false;
		double minAng;
		while (Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng * Math.PI / 180.0, false);
			this.setSpeeds(FAST, FAST);
			if(Main.usSensor.getFilteredDataBasic() < 10) {
				Navigation.PathBlocked = true;
				break;
			}
		}
		this.setSpeeds(0, 0);
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments
	 * @param angle - absolute angle to turn to
	 * @param stop - if the robot should stop moving after turning
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getTheta();
		
		while (Math.abs(error) > DEG_ERR * Math.PI / 180.0) {
			
			error = angle - this.odometer.getTheta();
			
			if((Math.abs(error)) > Math.PI) {
				if(error < 0) error += 2*Math.PI;
				else error -= 2*Math.PI;
			}
			
			
			if (error < -Math.PI) {
				this.setSpeeds(-SLOW, SLOW);
				//odometer.getMotors()[0].forward();
				//odometer.getMotors()[1].backward();
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
				//odometer.getMotors()[0].backward();
				//odometer.getMotors()[1].forward();
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
				//odometer.getMotors()[0].backward();
				//odometer.getMotors()[1].forward();
			} else {
				this.setSpeeds(-SLOW, SLOW);
				//odometer.getMotors()[0].forward();
				//odometer.getMotors()[1].backward();
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}
	
	/**
	 * turnBy function that turns by an angle relative to the robot
	 * @param theta - relative angle to turn by
	 */
	public void turnBy(double theta) {
		odometer.setMotorSpeeds(Odometer.ROTATE_SPEED, Odometer.ROTATE_SPEED);
		odometer.getMotors()[0].rotate(convertAngle(odometer.wheelRadius,odometer.trackLength,theta * 180.0 / Math.PI), true);
		odometer.getMotors()[1].rotate(-convertAngle(odometer.wheelRadius,odometer.trackLength,theta * 180.0 / Math.PI), false);
	}
	
	/**
	 * Converts linear distance to degrees a wheel must rotate
	 * @param radius - radius of the wheel (cm)
	 * @param distance - distance (cm)
	 * @return - degrees to rotate
	 */
	private static int convertDistance(double radius, double distance) {
		return (int)(distance * 180.0 / (Math.PI * radius));
	}

	/**
	 * Converts angle to rotate to degrees a wheel must rotate
	 * @param radius - radius of the wheel (cm)
	 * @param width - distance between wheels
	 * @param angle - angle to rotate
	 * @return - degrees to rotate
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Calculates minimal angle
	 * @param theta1 - in radians
	 * @param theta2 - in radians
	 * @return - minimal angle between theta1 and theta2
	 */
	public static double minimalAngle(double theta1, double theta2) {
		double dTheta = theta1 - theta2;
		if(dTheta > Math.PI) dTheta -= 2*Math.PI;
		else if (dTheta < -Math.PI) dTheta += 2*Math.PI;
		return dTheta;
	}
	
	/**
	 * Go forward a set distances
	 * @param distance - distance to move (cm)
	 */
	public void goForward(double distance) {
		this.travelTo(Math.cos(Math.toRadians(this.odometer.getTheta())) * distance, Math.cos(Math.toRadians(this.odometer.getTheta())) * distance);

	}
}
