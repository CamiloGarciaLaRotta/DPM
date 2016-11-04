/* 
 * OdometryCorrection.java
 */
package utilities;

import chassis.Main;
import chassis.LightIntensitySensor;

/**
 * @version 0.1
 * @author juliette
 * Corrects the odometer reading using the grid on the field
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
	private final boolean updateAll[] = {true, true, true};

	/**
	 * 
	 * @param odometer - odometer to correct
	 */
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		count = 0;
		lastPosition = new double[3]; //x, y, theta
		odometer.getPosition(lastPosition, updateAll);	//need to set theta - may as well get current x and y
	}

	// run method (required for Thread)
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
				System.out.println("         LINE");
				double currentPosition[] = new double[3];
				odometer.getPosition(currentPosition, updateAll);
				//check theta (see if we changed heading and turned)
				if(Math.abs(currentPosition[2] - lastPosition[2]) < THRESHOLD_THETA)
				{
					if(Math.abs(Util.SQUARE_LENGTH + lastPosition[1] - currentPosition[1]) < THRESHOLD_POS ||	//no line missed
							Math.abs(Util.SQUARE_LENGTH + lastPosition[0] - currentPosition[0]) < THRESHOLD_POS)
					{
						if(Math.abs(currentPosition[2] - 0.00) < THRESHOLD_THETA) //+x direction
							odometer.setX(lastPosition[0] + Util.SQUARE_LENGTH);
						else if(Math.abs(currentPosition[2] - (-Math.PI/2)) < THRESHOLD_THETA) //-y
							odometer.setY(lastPosition[1] - Util.SQUARE_LENGTH);
						else if(Math.abs(currentPosition[2] - (-Math.PI)) < THRESHOLD_THETA) //-x
							odometer.setX(lastPosition[0] - Util.SQUARE_LENGTH);
						else if(Math.abs(currentPosition[2] - (-Math.PI*3/2)) < THRESHOLD_THETA) //+y
							odometer.setY(lastPosition[1] + Util.SQUARE_LENGTH);
					}
				}
				//if nothing was done in the above block we either missed a line or turned and we just need to update
				lastPosition = currentPosition.clone();
				count++;
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
