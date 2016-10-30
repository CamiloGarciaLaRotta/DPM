package utilities;

import chassis.ColorSensor;
import chassis.LCDInfo;
import chassis.USSensor;

public class Util {
	public static LCDInfo lcd;

	public static final double SQUARE_LENGTH = 30.48;
	public static final double FIELD_BOUNDARY = 3.0 * SQUARE_LENGTH;

	public static void setLCD(LCDInfo lcd) {
		Util.lcd = lcd;
	}
}
