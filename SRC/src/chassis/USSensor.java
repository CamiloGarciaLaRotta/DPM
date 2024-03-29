package chassis;

/*
 * AUTHORS
 * Juliette Regimbal
 * Harley Wiltzer
 */

import java.util.Arrays;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @version 3.0
 * Provides access to USSensor functions.
 * Handles mutual exclusion.
 */
public class USSensor {
	private SensorModes usSensor;
	private float[] usData;
	private SampleProvider usValue;
	public static final float FIELD_BOUNDS = 120;
	
	private Object mutex;	//to avoid race conditions
	
	/**
	 * USSensor Constructor
	 * @param usPort port the sensor is connected to
	 */
	public USSensor(Port usPort) {
		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usValue = usSensor.getMode("Distance");
		this.usData = new float[usValue.sampleSize()];
		this.mutex = new Object();
	}
	
	/**
	 * Returns distance measured by the sensor with an upper limit
	 * @return - distance in cm capped at FIELD_BOUNDS
	 */
	public float getFilteredDataBasic() {
		synchronized(mutex) {
			usSensor.fetchSample(usData, 0); //Store distance in usData
		}
		return (usData[0] * 100.0f >= FIELD_BOUNDS) ? FIELD_BOUNDS : usData[0] * 100.0f; //Cap data at NO_WALL, scale data by 100.
	}
	
	/**
	 * Takes multiple samples and returns the median
	 * @param samples number of samples to take
	 * @return median sample taken using getFilteredDataBasic()
	 * @see #getFilteredDataBasic()
	 */
	public float getMedianSample(int samples) {
		float[] distSamples = new float[samples];
		
		for(int i = 0; i < samples; i++) {
			distSamples[i] = getFilteredDataBasic();
		}
		Arrays.sort(distSamples);
		return (samples%2==0 ? (distSamples[samples/2]+distSamples[samples/2 + 1])/2 : distSamples[samples/2]);
	}

}
