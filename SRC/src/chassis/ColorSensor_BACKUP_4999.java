package chassis;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorSensor {
	private SampleProvider colorSample;
	private float[] colorData;
	private EV3ColorSensor colorValue;
	
<<<<<<< HEAD
	private Object mutex;
	
	/**
	 * ColorSensor Constructor (RGB mode)
	 * @param colorPort - port sensor is connected to
	 */
=======
>>>>>>> added Test class, passed important constants to Util class
	public ColorSensor(Port colorPort) {
		this.colorValue = new EV3ColorSensor(colorPort);
		this.colorSample = colorValue.getMode("RGB");
		this.colorData = new float[colorSample.sampleSize()];
	}
	
	/**
	 * 
	 * @return - RGB data from color sensor
	 */
	public float[] getColor() {
		colorSample.fetchSample(colorData, 0);
		return colorData;
	}
	
}