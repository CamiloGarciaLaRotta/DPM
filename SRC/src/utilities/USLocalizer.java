package utilities;

import chassis.Main;
import chassis.USSensor;
import lejos.hardware.Sound;

/**
 * @version 0.2
 * Localize the robot in the first square to (0, 0, 90-degrees)
 * @author juliette
 *
 */
public class USLocalizer extends Thread {
	private static final float NO_WALL = 45.0f;	//Minimum distance for which the US sensor reading should be interpreted as no wall detected
	private static final double THETA_THRESHOLD = Math.PI / 6.0; //Minimum angle between two walls
	public static int ROTATION_SPEED = 60;
		
	private Odometer odo;
	private USSensor usSensor;
	private double lastTheta;
	int step;
	private double minimumDistance;
	private double distanceUSSensor;
	
	/**
	 * USLocalizer Constructor
	 * @param odometer - Odometer object
	 * @param usSensor - USSensor object
	 * @param distanceUSSensor - distance of sensor from center of rotation (in cm)
	 */
	public USLocalizer(Odometer odometer,  USSensor usSensor, double distanceUSSensor) {
		this.odo = odometer;
		this.usSensor = usSensor;
		this.distanceUSSensor = distanceUSSensor;
		step = 0;
		minimumDistance = NO_WALL;
	}
	
	/**
	 * Run to localize robot
	 */
	public void doLocalization() {
		Main.state = Main.RobotState.Localization;
		this.start();
	}
	
	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		double angleA, angleB;
		//localize();
					
		double distance;
		
		odo.getMotors()[0].setSpeed(ROTATION_SPEED);
		odo.getMotors()[1].setSpeed(ROTATION_SPEED);
		
		while(!seesWall()) {
			odo.getMotors()[0].forward();
			odo.getMotors()[1].backward();
		}
		odo.getMotors()[0].setSpeed(0);
		odo.getMotors()[1].setSpeed(0);
		lastTheta = odo.getTheta();

		// keep rotating until the robot sees a wall, then latch the angle
		while(seesWall() || Math.abs(odo.getTheta() - lastTheta) < THETA_THRESHOLD) {
			odo.getMotors()[0].setSpeed(ROTATION_SPEED);
			odo.getMotors()[1].setSpeed(ROTATION_SPEED);
			odo.getMotors()[0].forward();
			odo.getMotors()[1].backward();
			if((distance = usSensor.getMedianSample(5)) < minimumDistance) minimumDistance = distance;
			step++;
		}
		odo.getMotors()[0].setSpeed(0);
		odo.getMotors()[1].setSpeed(0);
		lastTheta = odo.getTheta();
		
		angleA = (odo.getTheta() < 0.0) ? odo.getTheta() + 2*Math.PI : odo.getTheta(); //Remove negative angles

		// switch direction and wait until it sees no wall
		while(!seesWall() || Math.abs(odo.getTheta() - lastTheta) < THETA_THRESHOLD) {
			odo.getMotors()[0].setSpeed(ROTATION_SPEED);
			odo.getMotors()[1].setSpeed(ROTATION_SPEED);
			odo.getMotors()[0].backward();
			odo.getMotors()[1].forward();
			step++;
		}
		step = 0;

		// keep rotating until the robot sees a wall, then latch the angle 
		while(seesWall() || step < 30) {
			odo.getMotors()[0].backward();
			odo.getMotors()[1].forward();
			if((distance = usSensor.getMedianSample(5)) < minimumDistance) minimumDistance = distance;
			step++;
		}
		step = 0;
		angleB = (odo.getTheta() < 0.0) ? odo.getTheta() + 2*Math.PI : odo.getTheta(); //Remove negative angles
		odo.getMotors()[0].stop();
		odo.getMotors()[1].stop();
		
		// angleA is clockwise from angleB, so assume the average of the
		// angles to the right of angleB is 45 degrees past 'north'
		double angle;
		if(angleA - angleB > 0) angle = (Math.PI/4) - 0.5 * (angleA + angleB); //Calculate heading corrections as seen in tutorial
		else angle = (5*Math.PI/4) - 0.5 * (angleA + angleB);
		
		// update the odometer position (example to follow:)
		odo.setPosition(new double [] {minimumDistance - Util.SQUARE_LENGTH + distanceUSSensor, minimumDistance - Util.SQUARE_LENGTH + distanceUSSensor,
				angle + odo.getTheta()}, new boolean [] {true, true, true}); //Update odometer values
		Navigation nav = new Navigation(odo);
		nav.travelTo(0,0);
		nav.turnTo(Math.PI/2, true);
		Sound.beepSequenceUp();
		
		//slight pause to show localization is complete
		try {
			Thread.sleep(2000);
		} catch (Exception e) {	}
	
		Main.state = Main.RobotState.Search;
	}
	
	/**
	 * Checks if the robot sees a wall for localizing
	 * @return - if a wall is seen
	 */
	private boolean seesWall() {
		float sample = usSensor.getMedianSample(10);
		if(sample < minimumDistance) minimumDistance = sample;
		return (sample < NO_WALL);
	}
	

}
