package utilities;

import chassis.Main;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import utilities.Search.SearchState;

public class Capture extends Thread {
	private Odometer odo;
	
	private final int ARM_SPEED	= 70;
	
	private EV3LargeRegulatedMotor leftArm;
	private EV3LargeRegulatedMotor rightArm;
	private Navigation nav;
	
	/**
	 * Capture Thread Constructor
	 * @param odometer - Odometer Object
	 * @param leftArm - motor for left arm
	 * @param rightArm - motor for right arm
	 */
	public Capture(Odometer odometer, EV3LargeRegulatedMotor leftArm, EV3LargeRegulatedMotor rightArm) {
		this.odo = odometer;
		this.leftArm = leftArm;
		this.rightArm = rightArm;
		this.nav = new Navigation(this.odo);
	}
	
	/**
	 * start Capture thread
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while(Main.state != Main.RobotState.Capture) {
			if(Thread.interrupted()) return;
			//wait for capture to begin
			try {
				Thread.sleep(300);
			} catch (InterruptedException e){ }
		}
		nav.turnBy(Math.PI);
		odo.moveCM(Odometer.LINEDIR.Backward, 3, true);
		
		getBlock();
		Navigation nav = new Navigation(odo); //travel to scoring zone with block
		nav.travelTo(Util.GOAL_ZONE[0] - 17,Util.GOAL_ZONE[1] - 17); 
		
		while(Navigation.PathBlocked) {
			if(Thread.interrupted()) return;
			odo.stopMotors();
			//If it can (without going out of bounds), turn clockwise and move 20cm
			if(inBounds(odo.getX() + 20*Math.cos(odo.getTheta() - Math.PI/2),odo.getY() - Math.sin(odo.getTheta() - Math.PI/2),60,60)) {
				nav.turnBy(Math.PI/2);
				odo.moveCM(Odometer.LINEDIR.Forward,20,true);
			}
			//Otherwise turn counterclockwise and move 20cm
			else {
				nav.turnBy(Math.PI/2);
				odo.moveCM(Odometer.LINEDIR.Forward, 20, true);
			}
			nav.travelTo(Util.GOAL_ZONE[0] - 17, Util.GOAL_ZONE[1] - 17); 
			nav.turnBy(Math.PI);
		}
		
		//TODO implement capture code
		odo.setMotorSpeeds(0, 0); //ensure motors are stopped
		odo.forwardMotors();
		nav.turnBy(Math.PI);

		Main.state = Main.RobotState.Search;
		Search.searchState = SearchState.Default;
		ascendArms();
		Sound.beep();
		Sound.beep();
		Sound.beep();
	}
	
	private void descendArms() {
		leftArm.setSpeed(ARM_SPEED);
		rightArm.setSpeed(ARM_SPEED);
		this.leftArm.rotate(90 + Main.RESTING_ARM_POSITION,true);
		this.rightArm.rotate(90 + Main.RESTING_ARM_POSITION,false);
	}
	
	private void ascendArms() {
		leftArm.setSpeed(ARM_SPEED);
		rightArm.setSpeed(ARM_SPEED);
		this.leftArm.rotate(-90,true);
		this.rightArm.rotate(-90,false);
	}
	
	private void getBlock() {
		//TODO CATCH BLOCK, avoid the obstacle if it lies in the path
		Sound.beep();
		descendArms();
	}
	
	/**
	 * Checks if the position is in field bounds
	 * @param x
	 * @param y
	 * @param width - field width (x)
	 * @param height - field height (y)
	 * @return
	 */
	public static boolean inBounds(double x,double y, double width, double height) {
		return (x < width) && (y < height) && x > 0 && y > 0;
	}
}
