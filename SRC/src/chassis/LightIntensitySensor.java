package chassis;

/*
 * AUTHORS
 * Juliette Regimbal
 * Harley Wiltzer
 */

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Provides access to Light Sensor functions
 * Handles mutual exclusion
 * @version 3.0
 * 
 */
public class LightIntensitySensor {
	private SampleProvider intensitySample;
	private float[] intensityData;
	private EV3ColorSensor intensityValue;

	public static final float LINE_DETECTED	= 300.0f;
	
	private Object mutex;
	
	/**
	 * LightIntensitySensor Constructor
	 * @param intensityPort port sensor is connected to
	 */
	public LightIntensitySensor(Port intensityPort) {
		this.intensityValue = new EV3ColorSensor(intensityPort);
		this.intensitySample = intensityValue.getMode("Red");
		this.intensityData = new float[intensitySample.sampleSize()];
		this.mutex = new Object();
	}
	
	/**
	 * Gets light intensity from the sensor scaled by 1000
	 * @return intensity light intensity * 1000
	 */
	public float getIntensity() {
		synchronized(mutex) {
			intensitySample.fetchSample(intensityData, 0);
		}
		return intensityData[0] * 1000.0f;
	}
	
}

