/* 
 * OdometryCorrection.java
 */
package utilities;

import chassis.Main;
import chassis.LightIntensitySensor;

/**
 * Corrects the odometer reading using the grid on the field
 * @version 3.0
 */
public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	
	private static final double THRESHOLD_THETA = 40.0 * Math.PI / 180.0; //threshold for theta
	private static final double THRESHOLD_POS = 5.0;	//threshold for x and y values
	public static int count;
	private double lastPosition[];
	private float lastIntensity;
	private final boolean useCorrection = false;

	/**
	 * 
	 * @param odometer - odometer to correct
	 */
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		count = 0;
		lastPosition = new double[3]; //x, y, theta
		odometer.getPosition(lastPosition, Util.UPDATE_ALL);	//need to set theta - may as well get current x and y
	}

	/**
	 * run method (required for Thread)
	 * {@inheritDoc}
	 */
	public void run() {
		if(!useCorrection)
		{
			return;
		}
		long correctionStart, correctionEnd;
		lastIntensity = Main.gridLineDetector.getIntensity();
		
		while (true) {
			float intensity;
			correctionStart = System.currentTimeMillis();
			intensity = Main.gridLineDetector.getIntensity();
			//Intensity at line is < 300.
			if(intensity < LightIntensitySensor.LINE_DETECTED && lastIntensity < LightIntensitySensor.LINE_DETECTED) continue; //Only register line when the last measurement DID NOT measure a line
			lastIntensity = intensity;

			// put your correction code here
			
			if(intensity < LightIntensitySensor.LINE_DETECTED) //Detected black
			{
				double currentPosition[] = new double[3];
				odometer.getPosition(currentPosition, Util.UPDATE_ALL);
				
				//check if vertical line...
				double distanceX = (currentPosition[0]/Util.SQUARE_LENGTH) - Math.floor(currentPosition[0]/Util.SQUARE_LENGTH);
				if(distanceX < Util.GRIDLINE_THRESHOLD) odometer.setX(Math.floor(currentPosition[0]/Util.SQUARE_LENGTH)*Util.SQUARE_LENGTH);
				
				//check if horizontal line...
				double distanceY = (currentPosition[1]/Util.SQUARE_LENGTH) - Math.floor(currentPosition[1]/Util.SQUARE_LENGTH);
				if(distanceY < Util.GRIDLINE_THRESHOLD) odometer.setY(Math.floor(currentPosition[1]/Util.SQUARE_LENGTH)*Util.SQUARE_LENGTH);
			}
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
}
