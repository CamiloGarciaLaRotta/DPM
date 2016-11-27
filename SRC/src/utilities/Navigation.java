package utilities;
import chassis.Main;
import lejos.hardware.Sound;
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
import utilities.Odometer.TURNDIR;

/**
 * Helpful tools to control robot navigation
 * @version 3.0
 * @author juliette
 *
 */
public class Navigation {
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
		this.leftMotor.setAcceleration(Util.NAV_ACCELERATION);
		this.rightMotor.setAcceleration(Util.NAV_ACCELERATION);
	}

	/**
	 * Function to set the motor speeds jointly
	 * @param lSpd - speed of left motor
	 * @param rSpd - speed of right motor
	 */
	
	//TODO: Only use of one setSpeeds(...) and cast args to floats
	
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
		minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()));
		double error = minAng - this.odometer.getTheta();
		if(error > Math.PI) error -= 2 * Math.PI;
		else if(error < -Math.PI) error += 2 * Math.PI;
		turnBy(-error);
		double distance;
		while ((distance = Odometer.euclideanDistance(	new double[] {odometer.getX(), odometer.getY()},
											new double[] {x,y})) > Util.CM_TOLERANCE) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()));
			//minAng = minimalAngle(odometer.getTheta(),minAng);
			//if(minAng > DEG_ERR*Math.PI/180) this.turnBy(minAng);
			if(distance > 3 * Util.CM_TOLERANCE) this.turnTo(minAng, false);
			this.setSpeeds(Util.MOTOR_FAST, Util.MOTOR_FAST);
			if(Main.usSensor.getFilteredDataBasic() < Util.AVOID_DISTANCE) {
				Navigation.PathBlocked = true;
//				Sound.beepSequence();
				break;
			}
		}
		this.setSpeeds(0,0);
	}
	
	public void travelTo(double x, double y, boolean checkSides) {
		Navigation.PathBlocked = false;
		double minAng;
		minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()));
		double error = minAng - this.odometer.getTheta();
		if(error > Math.PI) error -= 2*Math.PI;
		else if(error < -Math.PI) error += 2 * Math.PI;
		turnBy(-error);
		double distance;
		double[] lastPos = new double[] {odometer.getX(), odometer.getY()};
		while ((distance = Odometer.euclideanDistance(	new double[] {odometer.getX(), odometer.getY()},
											new double[] {x,y})) > Util.CM_TOLERANCE) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX()));
			//minAng = minimalAngle(odometer.getTheta(),minAng);
			//if(minAng > DEG_ERR*Math.PI/180) this.turnBy(minAng);
			if(distance > 3 * Util.CM_TOLERANCE) this.turnTo(minAng, false);
			this.setSpeeds(Util.MOTOR_FAST, Util.MOTOR_FAST);
			if(Main.usSensor.getFilteredDataBasic() < Util.AVOID_DISTANCE) {
				Navigation.PathBlocked = true;
//				Sound.beepSequence();
				break;
			}
			
			if(checkSides && Odometer.euclideanDistance(new double[] {odometer.getX(), odometer.getY()}, lastPos) > Util.TRAVELTO_BW) {	//check fov
				this.setSpeeds(0, 0);
				lastPos = new double[] {odometer.getX(), odometer.getY()};
				double currentHeading = odometer.getTheta();
				//always use avoid distance or travelto bandwidth - larger of the two
				double relativeAngle = Math.atan2(Util.AVOID_DISTANCE > Util.TRAVELTO_BW ? Util.AVOID_DISTANCE : Util.TRAVELTO_BW, Util.ROBOT_WIDTH/2);
				odometer.setMotorSpeed(USLocalizer.ROTATION_SPEED);
				this.turnTo(currentHeading + relativeAngle, true);	//total field of view to rotate through is 2*relativeAngle
				odometer.spin(TURNDIR.CW);
				while(Math.abs(odometer.getTheta() - (currentHeading - relativeAngle)) > Util.DEG_TOLERANCE) {
					if(Main.usSensor.getFilteredDataBasic() < Util.AVOID_DISTANCE) {
						this.setSpeeds(0, 0);
						Navigation.PathBlocked = true;
//						Sound.beepSequence();
						return;
					}
				}
				this.setSpeeds(0, 0);
			}
		}
		this.setSpeeds(0,0);
	}

	/**
	 * TurnTo function which takes an angle and boolean as arguments
	 * @param angle - absolute angle to turn to
	 * @param stop - if the robot should stop moving after turning
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getTheta();
		if(error > Math.PI) error -= 2 * Math.PI;
		else if(error < -Math.PI) error += 2 * Math.PI;
		
		while (Math.abs(error) > Util.DEG_TOLERANCE * Math.PI / 180.0) {
			
			error = angle - this.odometer.getTheta();
			
			if((Math.abs(error)) > Math.PI) {
				if(error < 0) error += 2*Math.PI;
				else error -= 2*Math.PI;
			}
			
			
			if (error < -Math.PI) {
				this.setSpeeds(-Util.MOTOR_SLOW, Util.MOTOR_SLOW);
				//odometer.getMotors()[0].forward();
				//odometer.getMotors()[1].backward();
			} else if (error < 0.0) {
				this.setSpeeds(Util.MOTOR_SLOW, -Util.MOTOR_SLOW);
				//odometer.getMotors()[0].backward();
				//odometer.getMotors()[1].forward();
			} else if (error > Math.PI) {
				this.setSpeeds(Util.MOTOR_SLOW, -Util.MOTOR_SLOW);
				//odometer.getMotors()[0].backward();
				//odometer.getMotors()[1].forward();
			} else {
				this.setSpeeds(-Util.MOTOR_SLOW, Util.MOTOR_SLOW);
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
		odometer.getMotors()[0].rotate(convertAngle(Util.WHEEL_RADIUS,Util.TRACK,theta * 180.0 / Math.PI), true);
		odometer.getMotors()[1].rotate(-convertAngle(Util.WHEEL_RADIUS,Util.TRACK,theta * 180.0 / Math.PI), false);
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
