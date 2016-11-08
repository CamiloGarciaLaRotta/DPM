package utilities;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class LightIntensitySensor {
	private SampleProvider intensitySample;
	private float[] intensityData;
	private EV3ColorSensor intensityValue;

	public static final float LINE_DETECTED	= 300.0f;
	
	public LightIntensitySensor(Port intensityPort) {
		this.intensityValue = new EV3ColorSensor(intensityPort);
		this.intensitySample = intensityValue.getMode("Red");
		this.intensityData = new float[intensitySample.sampleSize()];
	}
	
	public float getIntensity() {
		intensitySample.fetchSample(intensityData, 0);
		return intensityData[0] * 1000.0f;
	}
	
}

