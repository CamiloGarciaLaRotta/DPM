package utilities;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
	public double trackLength; 
	public double wheelRadius;
	private long odometerPeriod;

	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, long odometerPeriod, double wheelRadius, double track) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.trackLength = track;
		this.wheelRadius = wheelRadius;
		this.odometerPeriod = odometerPeriod;
		this.x = this.y = 0.0;
		this.theta = 0.5 * Math.PI;
		this.leftMotorTachoCount = this.rightMotorTachoCount = 0;
		this.oldltacho = this.oldrtacho = 0;
		this.leftMotor.resetTachoCount();
		this.rightMotor.resetTachoCount();
		mutex = new Object();
	}

	@Override
	public void run() {
		long updateStart, updateEnd;
		
		while(true) {
			updateStart = System.currentTimeMillis();
			this.leftMotorTachoCount = this.leftMotor.getTachoCount();
			this.rightMotorTachoCount = this.rightMotor.getTachoCount();
								
			double dLeft, dRight, dTheta, dPos; //left wheel movement, right wheel movement, change in heading, and change in position
			dLeft = wheelRadius * Math.PI * (this.leftMotorTachoCount - this.oldltacho) / 180.0;
			dRight = wheelRadius * Math.PI * (this.rightMotorTachoCount - this.oldrtacho) / 180.0;
			oldltacho = leftMotorTachoCount;
			oldrtacho = rightMotorTachoCount;
			dTheta = (-dLeft + dRight)/trackLength;
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
			if (updateEnd - updateStart < odometerPeriod) {
				try {
					Thread.sleep(odometerPeriod - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}
	
	public EV3LargeRegulatedMotor[] getMotors() {
		EV3LargeRegulatedMotor[] motors = {leftMotor, rightMotor};
		return motors;
	}
	
	public void setMotorSpeeds(int speedL, int speedR) {
		leftMotor.setSpeed(speedL);
		rightMotor.setSpeed(speedR);
	}
	
	public void setMotorSpeed(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}
	
	public void forwardMotors() {
		leftMotor.forward();
		rightMotor.forward();
	}
	
	public void backwardMotors() {
		leftMotor.backward();
		rightMotor.backward();
	}
	
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
	
	public void stopMotors() {
		setMotorSpeed(0);
		forwardMotors();
	}
	
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

	public void getPosition(double[] position, boolean[] update) {
		synchronized(mutex) {
			if(update[0]) position[0] = x;
			if(update[1]) position[1] = y;
			if(update[2]) position[2] = theta;
		}
	}
	
	public void getPosition(double[] position) {
		synchronized(mutex) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}

	public double getX() {
		double result;
		synchronized(mutex) {
			result = x;
		}
		return result;
	}

	public double getY() {
		double result;
		synchronized(mutex) {
			result = y;
		}
		return result;
	}

	public double getTheta() {
		double result;
		synchronized(mutex) {
			result = theta;
		}
		return result;
	}

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

	public void setX(double x) {
		synchronized (mutex) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (mutex) {
			this.y = y;
		}
	}

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
	
	public double getFilteredData() {
		return data;
	}
	
	public void setData(double data) {
		this.data = data;
	}
	
	public static double euclideanDistance(double[] a, double[] b) {
		return Math.sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[1]-b[1])*(a[1]-b[1]));
	}
	
	/*
	public void turnTo(double theta) {
		if(Math.abs(theta - getTheta()) > THETA_THRESHOLD) {
			double adjustment = theta - getTheta();
			if(Math.abs(adjustment) > Math.PI) adjustment -= Math.PI*2;
			if(!(Math.abs(theta - getTheta()) < Math.PI)) {
				if(theta - getTheta() < 0.0) theta = theta + Math.PI;
				else theta = theta - Math.PI;
			} 
			turnBy(adjustment);
			Sound.beep();
		}
	}
	
	public void turnBy(double theta) {
		setMotorSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		leftMotor.rotate(convertAngle(wheelRadius,trackLength,theta * 180.0 / Math.PI), true);
		rightMotor.rotate(-convertAngle(wheelRadius,trackLength,theta * 180.0 / Math.PI), false);
	}
	
	public void travelTo(double x, double y) {
		while(euclideanDistance(new double [] {x, y}, new double [] {getX(), getY()}) > NAVIGATE_THRESHOLD) {			
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
	
	private int convertDistance(double radius, double distance) {
		return (int)(distance * 180.0 / (Math.PI * radius));
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
*/
	
}

