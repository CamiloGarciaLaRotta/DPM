package chassis;

import java.util.Arrays;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class USSensor {
	private SensorModes usSensor;
	private float[] usData;
	private SampleProvider usValue;
	public static final float FIELD_BOUNDS = 120;
	
	public USSensor(Port usPort) {
		this.usSensor = new EV3UltrasonicSensor(usPort);
		this.usValue = usSensor.getMode("Distance");
		this.usData = new float[usValue.sampleSize()];
	}
	
	public float getFilteredDataBasic() {
		usSensor.fetchSample(usData, 0); //Store distance in usData
		return (usData[0] * 100.0f >= FIELD_BOUNDS) ? FIELD_BOUNDS : usData[0] * 100.0f; //Cap data at NO_WALL, scale data by 100.
	}
	
	public float getMedianSample(int samples) {
		float[] distSamples = new float[samples];
		
		for(int i = 0; i < samples; i++) {
			distSamples[i] = getFilteredDataBasic();
		}
		Arrays.sort(distSamples);
		return (samples%2==0 ? (distSamples[samples/2]+distSamples[samples/2 + 1])/2 : distSamples[samples/2]);
	}

}
