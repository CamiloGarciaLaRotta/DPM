package chassis;

import java.io.IOException;
import java.util.HashMap;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import utilities.Avoider;
import utilities.Capture;
import utilities.Navigation;
import utilities.Odometer;
import utilities.Search;
import utilities.Test;
import utilities.ThreadEnder;
import utilities.USLocalizer;
import utilities.Util;
import utilities.Avoider;
import utilities.Capture;
import utilities.Navigation;
import wifi.WifiConnection;

/**
 * Base robot class with all hardware objects and loaded utilities
 * @version 0.2
 * @author juliette
 *
 */
public class Main {
	//Wifi
	private static WifiConnection conn = null;
	private static HashMap<String, Integer> transmission = null;
	
	//Resources (motors, sensors)
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor forkliftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3MediumRegulatedMotor clawMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");
	private static final Port intensityPort = LocalEV3.get().getPort("S3");
	private static TextLCD textLCD = LocalEV3.get().getTextLCD();
	public static USSensor usSensor = new USSensor(usPort);
	public static ColorSensor colorSensor = new ColorSensor(colorPort);

	public static Forklift forklift;

	public static LightIntensitySensor gridLineDetector;
	
	public static RobotState state = RobotState.Disabled;
	public static RobotState lastState = RobotState.Disabled;
	public static DemoState demo = DemoState.Default;
	public static RobotTask task;
	public static int startingCorner;
	public static double[][] GREEN;
	public static double[][] RED;
	
	/**
	 * Current action the robot is doing
	 */
	public enum RobotState {Setup, Localization, Search, Capture, Disabled, Avoiding};
	/**
	 * 
	 * @author juliette
	 * Select test to run or run in match mode (Default).
	 */
	public enum DemoState {Default, StraightLineTest, SquareTest, LocalizationTest, NavigationTest, SearchTest, RGBVectorTest, TrackTest, ForkliftTest, Avoidance};	//can be expanded to include alternate options, debugging, hardware tests, etc.
	
	public enum RobotTask {Builder, Collector};
	
	public static LCDInfo lcd;
	
	public static final int RESTING_ARM_POSITION = 30;
	
	//TODO TODO TODO TODO
	// - implement WIFI module
	// - dynamically set GREEN, RED zone

	/**
	 * Main execution thread.
	 * @param args - None used
	 * @throws IOException 
	 */
	public static void main(String[] args) {
		state = RobotState.Setup;
		
		//Setup threads
		Odometer odo = new Odometer(leftMotor, rightMotor);
		Navigation nav = new Navigation(odo);
		lcd = new LCDInfo(odo, textLCD, false);	//do not start on creation
		ThreadEnder ender = new ThreadEnder();
		USLocalizer localizer = new USLocalizer(odo, usSensor, Util.US_TO_CENTER);
		forklift = new Forklift(forkliftMotor,clawMotor);
		
		// for testing only, when WIFI module is implemented it will be given automatically
		
		if(Util.USE_WIFI) {
			GREEN = new double[2][2];
			RED = new double[2][2];
			HashMap<String, Integer> parameters = null;
			try {
				parameters = wifiConnect();
			} catch (IOException e) {	//failed to connect to wifi
				System.err.println(e);
				System.exit(-1);
			}
			if(parameters != null) {
				transmissionParse(parameters);
			}
		} else {	//default parameters
			GREEN = new double[][]{{1*Util.SQUARE_LENGTH,1*Util.SQUARE_LENGTH},
				{3*Util.SQUARE_LENGTH,2*Util.SQUARE_LENGTH}};
			RED = new double[][]{{0*Util.SQUARE_LENGTH,5*Util.SQUARE_LENGTH},
					{2*Util.SQUARE_LENGTH,9*Util.SQUARE_LENGTH}};
			startingCorner = 1;
			task = RobotTask.Builder;
		}
		
		Search search = new Search(odo, colorSensor, usSensor, GREEN);
		Capture capture = new Capture(odo, GREEN);
		Avoider avoid = new Avoider(odo, nav, usSensor, RED);
		
		textLCD.clear(); //blank display before selection
		demo = stateSelect();	//select state
		
		//threads intrinsic to all processes
		odo.start();
		ender.start();
		lcd.resume();
		
		switch (demo) {
		case Default: //regular robot operation
			gridLineDetector = new LightIntensitySensor(intensityPort);
			
			localizer.doLocalization();
			search.start();
			avoid.start();
			capture.start();
			break;
			
		// Tests need to be verified in this order, 
		// as a test builds on top of the prior one.
		case StraightLineTest:	
			Test.StraightLineTest(odo, Util.SQUARE_LENGTH); // test tachometer/odometer
			break;
		case SquareTest:
			Test.SquareTest(odo, 3, 2 * Util.SQUARE_LENGTH); //test rotation
			break;
		case LocalizationTest:
			Test.LocalizationTest(odo); //test US sensor
			break;
		case NavigationTest:
			// the given points test all major rotation angles: 45, 135, 180, 360. Modify as needed
			Test.NavigationTest(odo, new int[][] {{60, 60}, {60,0}, {30,30}, {60,0}}, true); 
			break;
		case SearchTest:
			search.start();
			capture.start();
			state = RobotState.Search;
			break;
		case RGBVectorTest:
			colorSensor = new ColorSensor(colorPort);
			Test.RGBUnitVectorTest(colorSensor);
			break;
		case TrackTest:
			Test.TrackMeasureTest(odo, 10);
			break;
		case ForkliftTest:
			Test.ForkliftTest();
			break;
		case Avoidance:
			break;
		default:
			System.exit(-1);
		}
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);	//wait for escape key to end program
		
		//TODO if setting interrupt flag doesn't stop Threads,
		//	   add while(!Thread.interrupted()) at the 
		//	   beggining of each Thread's run() method
		odo.interrupt();
		search.interrupt();
		avoid.interrupt();
		capture.interrupt();
		localizer.interrupt();
		System.exit(0);
	}
	
	/**
	 * Creates menu to select a demo state
	 * @return Selected demo state
	 */
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
	
	private static void transmissionParse(HashMap<String, Integer> transmission) {
		if(transmission != null) {
			//Get task
			if(transmission.get("BTN") == Util.TEAM_NUMBER) {
				task = RobotTask.Builder;
			} else {
				task = RobotTask.Collector;
			}
			
			//Get assigned starting corner
			if(task == RobotTask.Builder) {
				startingCorner = transmission.get("BSC");
			} else {
				startingCorner = transmission.get("CSC");
			}
			
			//Get zone coordinates
			RED[0][0] = transmission.get("LRZx");
			RED[0][1] = transmission.get("LRZy");
			RED[1][0] = transmission.get("URZx");
			RED[1][1] = transmission.get("URZy");
			
			GREEN[0][0] = transmission.get("LGZx");
			GREEN[0][1] = transmission.get("LGZy");
			GREEN[1][0] = transmission.get("UGZx");
			GREEN[1][1] = transmission.get("UGZy");
		}
	}
	
	private static HashMap<String, Integer> wifiConnect() throws IOException {
		System.out.println("Connecting to "+Util.IP_ADDR);
		conn = new WifiConnection(Util.IP_ADDR, Util.TEAM_NUMBER, true);
		
		if(conn != null) {
			HashMap<String, Integer> t = conn.StartData;
			if(t == null) {
				System.out.println("Failed to get transmission data.");
			} else {
				System.out.println("Transmission received");
				return t;
			}
		}
		return null;
	}
}
