package chassis;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorSensor {
	private SampleProvider colorSample;
	private float[] colorData;
	private EV3ColorSensor colorValue;
	
	public ColorSensor(Port colorPort) {
		this.colorValue = new EV3ColorSensor(colorPort);
		this.colorSample = colorValue.getMode("RGB");
		this.colorData = new float[colorSample.sampleSize()];
	}
	
	public float[] getColor() {
		colorSample.fetchSample(colorData, 0);
		return colorData;
	}
	
}