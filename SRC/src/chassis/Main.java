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

/**
 * Base robot class with all hardware objects and loaded utilities
 * @version 0.1
 * @author juliette
 *
 */
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
	
	public static RobotState state = RobotState.Disabled;
	public static RobotState lastState = RobotState.Disabled;
	public static DemoState demo = DemoState.Default;
	
	/**
	 * Current action the robot is doing
	 */
	public enum RobotState {Setup, Localization, Search, Capture, Disabled, Avoiding};
	public enum DemoState {Default};	//can be expanded to include alternate options, debugging, hardware tests, etc.
	
	public static LCDInfo lcd;
	
	public static final int RESTING_ARM_POSITION	= 30;
	

	public static void main(String[] args) {
		state = RobotState.Setup;
		
		//Setup sensors
		usSensor = new USSensor(usPort);
		colorSensor = new ColorSensor(colorPort);
		gridLineDetector = new LightIntensitySensor(intensityPort);
		
		//Setup threads
		Odometer odo = new Odometer(leftMotor, rightMotor, ODOMETER_PERIOD, WHEEL_RADIUS, TRACK);
		lcd = new LCDInfo(odo, textLCD, false);	//do not start on creation
		USLocalizer localizer = new USLocalizer(odo, usSensor, US_TO_CENTER);
		Search search = new Search(odo, colorSensor, usSensor);
		Capture capture = new Capture(odo,leftArmMotor,rightArmMotor);
		
		textLCD.clear(); //blank display before selection
		demo = stateSelect();	//select state
		
		odo.start();
		lcd.resume();
		localizer.doLocalization();
		search.start();
		capture.start();
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);	//wait for escape key to end program
		
		odo.interrupt();;
		search.interrupt();
		capture.interrupt();
		localizer.interrupt();
		System.exit(0);
	}
	
	private static DemoState stateSelect() {
		DemoState state = DemoState.Default;
		DemoState [] states = DemoState.values();
		boolean stateChosen = false;
		while(!stateChosen) {
			LCDInfo.displayMessage("<- "+state.toString() + " ->");
			int selection = Button.waitForAnyPress();
			switch(selection) {
			case Button.ID_ENTER:
				stateChosen = true;
				break;
			case Button.ID_RIGHT:
				state = states[(state.ordinal() + 1) % states.length]; //increment state
				break;
			case Button.ID_LEFT:
				state = states[(state.ordinal() - 1) + (state.ordinal() == 0 ? states.length : 0)]; //decrement with wrap around
				break;
			default:
				System.exit(-1);
				break;
			}
		}
		
		return state;
	}
}
