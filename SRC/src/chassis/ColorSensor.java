package chassis;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Provides access to ColorSensor functions in RGB mode.
 * Handles mutual exclusion
 * @version 3.0
 * 
 */
public class ColorSensor {
	private SampleProvider colorSample;
	private float[] colorData;
	private EV3ColorSensor colorValue;
	
	private Object mutex;
	
	/**
	 * ColorSensor Constructor (RGB mode)
	 * @param colorPort port sensor is connected to
	 */
	public ColorSensor(Port colorPort) {
		this.colorValue = new EV3ColorSensor(colorPort);
		this.colorSample = colorValue.getMode("RGB");
		this.colorData = new float[colorSample.sampleSize()];
		this.mutex = new Object();
	}
	
	/**
	 * Get color from sensor
	 * @return RGB data from color sensor
	 */
	public float[] getColor() {
		synchronized(mutex) {
			colorSample.fetchSample(colorData, 0);
		}
		return colorData;
	}
	
}
