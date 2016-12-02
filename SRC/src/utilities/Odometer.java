package utilities;

/*
 * AUTHORS
 * Harley Wiltzer
 * Juliette Regimbal
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Odometer class for the robot
 * All values in cm and radians unless otherwise specified.
 * @version 3.0
 */
public class Odometer extends Thread {
	
	public enum TURNDIR {CW, CCW};
	public enum LINEDIR {Forward,Backward}

	public static final int ROTATE_SPEED = 150;
	public static final int NAVIGATE_SPEED = 100;
	
	private double x,y,theta;
	private int leftMotorTachoCount,rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	private Object mutex;

	private double oldltacho, oldrtacho;
	
	private double data;

	/**
	 * Odometer Constructor
	 * @param leftMotor left motor object
	 * @param rightMotor right motor object
	 */
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		this.x = this.y = 0.0;
		this.theta = 0.5 * Math.PI;
		this.leftMotorTachoCount = this.rightMotorTachoCount = 0;
		this.oldltacho = this.oldrtacho = 0;
		this.leftMotor.resetTachoCount();
		this.rightMotor.resetTachoCount();
		mutex = new Object();
	}

	/**
	 * Start odometer
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		long updateStart, updateEnd;
		
		while(true) {
			updateStart = System.currentTimeMillis();
			this.leftMotorTachoCount = this.leftMotor.getTachoCount();
			this.rightMotorTachoCount = this.rightMotor.getTachoCount();
								
			double dLeft, dRight, dTheta, dPos; //left wheel movement, right wheel movement, change in heading, and change in position
			dLeft = Util.WHEEL_RADIUS * Math.PI * (this.leftMotorTachoCount - this.oldltacho) / 180.0;
			dRight = Util.WHEEL_RADIUS * Math.PI * (this.rightMotorTachoCount - this.oldrtacho) / 180.0;
			oldltacho = leftMotorTachoCount;
			oldrtacho = rightMotorTachoCount;
			dTheta = (-dLeft + dRight)/Util.TRACK;
			dPos = (dLeft + dRight)/2;
			synchronized (mutex) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here!
				 * Only update the values of x, y, and theta in this block. 
				 * Do not perform complex math
				 * 
				 */
				theta += dTheta + 2*Math.PI;
				theta %= 2*Math.PI;	//radians - wrap around
				//if(theta < 0.0) theta += 2*Math.PI; //No negative angles
				y += dPos*Math.sin(theta);
				x += dPos*Math.cos(theta);
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < Util.ODOMETER_PERIOD) {
				try {
					Thread.sleep(Util.ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}
	
	/**
	 * Returns the motors used by the odometer
	 * @return array of motors (left, right)
	 */
	public EV3LargeRegulatedMotor[] getMotors() {
		EV3LargeRegulatedMotor[] motors = {leftMotor, rightMotor};
		return motors;
	}
	
	/**
	 * Update motor speeds (deg/s)
	 * @param speedL left speed
	 * @param speedR right sped
	 */
	public void setMotorSpeeds(int speedL, int speedR) {
		leftMotor.setSpeed(speedL);
		rightMotor.setSpeed(speedR);
	}
	
	/**
	 * Update motor speeds (deg/s)
	 * @param speed speed for both motors
	 */
	public void setMotorSpeed(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}
	
	/**
	 * Have both motors move forward
	 */
	public void forwardMotors() {
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * Have both motors move in reverse
	 */
	public void backwardMotors() {
		leftMotor.backward();
		rightMotor.backward();
	}
	
	/**
	 * Spin in a certain direction
	 * @param dir direction to spin in
	 */
	public void spin(TURNDIR dir) {
		if(dir == TURNDIR.CW) {
			leftMotor.forward();
			rightMotor.backward();
		}
		else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}
	
	/**
	 * Stop motors safely
	 */
	public void stopMotors() {
		setMotorSpeed(0);
		forwardMotors();
	}
	
	/**
	 * Move a set distance
	 * @param dir direction to move in
	 * @param distance distance to move (cm)
	 * @param stop if the robot should stop after moving
	 */
	public void moveCM(LINEDIR dir, double distance, boolean stop) {
		setMotorSpeeds(NAVIGATE_SPEED, NAVIGATE_SPEED);
		if(dir == LINEDIR.Forward) forwardMotors();
		else backwardMotors();
		double startpos[] = new double[3];
		double curpos[] = new double[3];
		getPosition(startpos,new boolean[] {true,true,true});
		do {
			getPosition(curpos, new boolean[] {true,true,true});
		}
		while(euclideanDistance(startpos,curpos) < distance);
		
		if(stop) {
			setMotorSpeed(0);
			forwardMotors();
		}
	}

	/**
	 * Write coordinates of the current position to an array
	 * @param position array to hold x, y, and theta coordinates
	 * @param update if the x, y, and theta coordinates written to the array
	 */
	public void getPosition(double[] position, boolean[] update) {
		synchronized(mutex) {
			if(update[0]) position[0] = x;
			if(update[1]) position[1] = y;
			if(update[2]) position[2] = theta;
		}
	}
	
	/**
	 * Write coordinates of current position to position
	 * @param position array to hold x, y, and theta coordinates
	 */
	public void getPosition(double[] position) {
		synchronized(mutex) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	/**
	 * 
	 * @return x coordinate from odometer
	 */
	public double getX() {
		double result;
		synchronized(mutex) {
			result = x;
		}
		return result;
	}

	/**
	 * 
	 * @return y coordinate from odometer
	 */
	public double getY() {
		double result;
		synchronized(mutex) {
			result = y;
		}
		return result;
	}

	/**
	 * 
	 * @return heading from odometer
	 */
	public double getTheta() {
		double result;
		synchronized(mutex) {
			result = theta;
		}
		return result;
	}

	/**
	 * Overrides the current position in the odometer
	 * @param position x, y, and theta overrides
	 * @param update if x, y, and theta should be overridden
	 */
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (mutex) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * 
	 * @param x x coordinate override
	 */
	public void setX(double x) {
		synchronized (mutex) {
			this.x = x;
		}
	}

	/**
	 * 
	 * @param y y coordinate override
	 */
	public void setY(double y) {
		synchronized (mutex) {
			this.y = y;
		}
	}

	/**
	 * 
	 * @param theta heading override
	 */
	public void setTheta(double theta) {
		synchronized (mutex) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (mutex) {
			this.leftMotorTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (mutex) {
			this.rightMotorTachoCount = rightMotorTachoCount;	
		}
	}
	
	/**
	 * @deprecated
	 * @return - data
	 */
	public double getFilteredData() {
		return data;
	}
	
	/**
	 * @deprecated
	 * @param data - data to set
	 */
	public void setData(double data) {
		this.data = data;
	}
	
	/**
	 * Euclidean distance between two (x, y) points
	 * @param a point a
	 * @param b point b
	 * @return euclidean distance
	 */
	public static double euclideanDistance(double[] a, double[] b) {
		return Math.sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1]-b[1])*(a[1]-b[1]));
	}
	
	/**
	 * TurnTo function. Use function in Navigation.
	 * @deprecated
	 * @see Navigation#turnTo(double, boolean)
	 */
	public void turnTo(double theta) {
		if(Math.abs(theta - getTheta()) > Util.DEG_TOLERANCE) {
			double adjustment = theta - getTheta();
			if(Math.abs(adjustment) > Math.PI) adjustment -= Math.PI*2;
			if(!(Math.abs(theta - getTheta()) < Math.PI)) {
				if(theta - getTheta() < 0.0) theta = theta + Math.PI;
				else theta = theta - Math.PI;
			} 
			turnBy(adjustment);
			//Sound.beep();
		}
	}
	
	/**
	 * TurnBy function. Use function in Navigation.
	 * @deprecated
	 * @see Navigation#turnBy(double)
	 */
	public void turnBy(double theta) {
		setMotorSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Util.WHEEL_RADIUS,Util.TRACK,theta * 180.0 / Math.PI), true);
		rightMotor.rotate(-convertAngle(Util.WHEEL_RADIUS,Util.TRACK,theta * 180.0 / Math.PI), false);
	}
	
	/**
	 * TravelTo function. Use function in Navigation.
	 * @deprecated
	 * @see Navigation#travelTo(double, double)
	 */
	public void travelTo(double x, double y) {
		while(euclideanDistance(new double [] {x, y}, new double [] {getX(), getY()}) > Util.CM_TOLERANCE) {			
			double dx = x - getX();
			double dy = y - getY();
			double theta = Math.atan2(dy,dx);
			turnTo(theta);
			
			leftMotor.setSpeed(NAVIGATE_SPEED);
			rightMotor.setSpeed(NAVIGATE_SPEED);
			leftMotor.forward();
			rightMotor.forward();
			
		}
		rightMotor.stop();
		leftMotor.stop();
	}
	
	/**
	 * @deprecated
	 * @param radius - wheel radius
	 * @param distance - linear distance
	 * @return - degrees to turn
	 * Replaced by method in Navigation
	 */
	private int convertDistance(double radius, double distance) {
		return (int)(distance * 180.0 / (Math.PI * radius));
	}

	/**
	 * @deprecated
	 * @param radius - wheel radius
	 * @param width - track width
	 * @param angle - angle to turn
	 * @return - degrees to rotate each wheel
	 * Replaced by method in Navigation
	 */
	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}

