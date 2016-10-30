package utilities;

import chassis.ColorSensor;
import chassis.USSensor;

public class Util {
	public static ColorSensor colorSensor;
	public static USSensor usSensor;
	public static LCDInfo lcd;

	public static final double SQUARE_LENGTH = 30.48;
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;

	public static void setColorSensor(ColorSensor colorSensor) {
		Util.colorSensor = colorSensor;
	}

	public static void setUSSensor(USSensor usSensor) {
		Util.usSensor = usSensor;
	}

	public static void setLCD(LCDIndo lcd) {
		Util.lcd = lcd;
	}
}
