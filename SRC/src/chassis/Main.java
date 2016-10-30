package chassis;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;

import utilities.Odometer;
import utilities.USLocalizer;
import utilities.Search;
import utilities.Capture;
import utilities.Util;


public class Main {
	//Constants (measurements, frequencies, etc)
	private static final long ODOMETER_PERIOD = 25;
	private static final double WHEEL_RADIUS = 2.141; //cm
	private static final double TRACK = 16.50; //cm (16.50 previously)
	private static final double US_TO_CENTER = 4.50; //cm from us sensor to center of wheels
	//Resources (motors, sensors)
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor leftArmMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor rightArmMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	private static final Port intensityPort = LocalEV3.get().getPort("S3");
	private static TextLCD textLCD = LocalEV3.get().getTextLCD();
	public static USSensor usSensor;
	public static ColorSensor colorSensor;
	public static LightIntensitySensor gridLineDetector;
	
	public static RobotState state = RobotState.k_Disabled;
	public static RobotState lastState = RobotState.k_Disabled;
	public static DemoState demo = DemoState.k_Default;
	
	public enum RobotState {k_Setup, k_Localization, k_Search, k_Capture, k_Disabled, k_Avoiding};
	public enum DemoState {k_Part1, k_Part2, k_Default};
	
	public static LCDInfo lcd;
	
	public static final int RESTING_ARM_POSITION	= 30;
	
	public static final double GRID_SIZE		= 30.48;
	public static final double FIELD_BOUNDARY	= 58;

	public static void main(String[] args) {
		state = RobotState.k_Setup;
		//Setup sensors
		usSensor = new USSensor(usPort);
		
		//leftArmMotor.rotate(-RESTING_ARM_POSITION);
		//rightArmMotor.rotate(-RESTING_ARM_POSITION);
		
		colorSensor = new ColorSensor(colorPort);
		gridLineDetector = new LightIntensitySensor(intensityPort);
		//Setup threads
		Odometer odo = new Odometer(leftMotor, rightMotor, ODOMETER_PERIOD, WHEEL_RADIUS, TRACK);
		lcd = new LCDInfo(odo, textLCD, false);	//start on creation
		USLocalizer localizer = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.RISING_EDGE, US_TO_CENTER);
		Search search = new Search(odo, colorSensor, usSensor);
		Capture capture = new Capture(odo,leftArmMotor,rightArmMotor);
		
		textLCD.clear();
		textLCD.drawString("<-Part 1 Part 2->", 0, 5);
		int input = Button.waitForAnyPress();
		textLCD.clear(5);	//clear option display
		lcd.resume();
		
		switch(input) {
		case Button.ID_RIGHT:
			demo = DemoState.k_Part2;
			state = RobotState.k_Search;
			odo.start();	//start threads
			localizer.doLocalization();
			capture.start();
			search.start();
			lcd.resume();
			break;
		case Button.ID_LEFT:
			demo = DemoState.k_Part1;
			state = RobotState.k_Search;
			LCDInfo.displayMessage("Part 1 started");
			search.start();
			break;
		case Button.ID_ENTER:
			odo.start();
			lcd.resume();
			leftMotor.flt();
			rightMotor.flt();
			break;
		default:
			//invalid input
			System.exit(-1);
		}
		//Wait for escape to exit
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);
		/*if(state == RobotState.k_Disabled) {	//execution has normally exited
			try {
				odo.join();
				search.join();
				capture.join();
				localizer.join();
			} catch (Exception e) {}
		} else {								//cancelled while still running
			try {
				odo.interrupt();
				search.interrupt();
				capture.interrupt();
				localizer.interrupt();
			} catch (Exception e) {}
		}*/
		odo.interrupt();;
		search.interrupt();
		capture.interrupt();
		localizer.interrupt();
		System.exit(0);
	}
}
