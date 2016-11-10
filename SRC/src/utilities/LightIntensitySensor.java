package utilities;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Provides LightIntensity functions
 * @version 0.2
 * @author juliette
 * 
 */
public class LightIntensitySensor {
	private SampleProvider intensitySample;
	private float[] intensityData;
	private EV3ColorSensor intensityValue;

	public static final float LINE_DETECTED	= 300.0f;
	
	/**
	 * Constructor for LightIntensitySensor
	 * @param intensityPort - port used by sensor
	 */
	public LightIntensitySensor(Port intensityPort) {
		this.intensityValue = new EV3ColorSensor(intensityPort);
		this.intensitySample = intensityValue.getMode("Red");
		this.intensityData = new float[intensitySample.sampleSize()];
	}
	
	/**
	 * Light intensity scaled up by 1000.
	 * @return Scaled light intensity measured.
	 */
	public float getIntensity() {
		intensitySample.fetchSample(intensityData, 0);
		return intensityData[0] * 1000.0f;
	}
	
}

