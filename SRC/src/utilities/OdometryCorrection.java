/* 
 * OdometryCorrection.java
 */
package utilities;

/*
 * AUTHORS
 * Harley Wiltzer
 */

import chassis.Main;
import chassis.LightIntensitySensor;

/**
 * Corrects the odometer reading using the grid on the field.
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
	private final boolean useCorrection = true;

	/**
	 * OdometryCorrection constructor
	 * @param odometer Odometer to correct
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
			float dI; //Stores the difference in intensity relative to last measurement
			correctionStart = System.currentTimeMillis();
			intensity = Main.gridLineDetector.getIntensity();
			dI = lastIntensity - intensity; //Get difference in two consecutive intensity values
			lastIntensity = intensity;

			if(dI > LightIntensitySensor.LINE_DETECTED) //Detected black
			{
				double currentPosition[] = new double[3];
				double sensorPosition[] = new double[2];
				//Use trig to find current position of light sensor
				odometer.getPosition(currentPosition, Util.UPDATE_ALL);
				sensorPosition[0] = currentPosition[0] - Util.INTENSITY_TO_CENTER * Math.cos(currentPosition[2]);
				sensorPosition[1] = currentPosition[1] - Util.INTENSITY_TO_CENTER * Math.sin(currentPosition[2]);
				
				//check if vertical line...
				double distanceX = (sensorPosition[0]/Util.SQUARE_LENGTH) - Math.floor(sensorPosition[0]/Util.SQUARE_LENGTH);
				if(distanceX < Util.GRIDLINE_THRESHOLD) {
					double sensorX = Math.floor(sensorPosition[0]/Util.SQUARE_LENGTH) * Util.SQUARE_LENGTH;
					odometer.setX(sensorX + Util.INTENSITY_TO_CENTER * Math.cos(currentPosition[0]));
				}
				
				//check if horizontal line...
				double distanceY = (sensorPosition[1]/Util.SQUARE_LENGTH) - Math.floor(sensorPosition[1]/Util.SQUARE_LENGTH);
				if(distanceY < Util.GRIDLINE_THRESHOLD) {
					double sensorY = Math.floor(sensorPosition[1]/Util.SQUARE_LENGTH) * Util.SQUARE_LENGTH;
					odometer.setY(sensorY + Util.INTENSITY_TO_CENTER * Math.sin(currentPosition[1]));
				}
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
