package chassis;

/*
 * AUTHORS
 * Camilo Garcia La Rotta
 * Harley Wiltzer
 * Juliette Regimbal
 */

import java.io.IOException;
import java.util.HashMap;

import lejos.hardware.Button;
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
import utilities.Search.SearchState;
import utilities.Test;
import utilities.ThreadEnder;
import utilities.USLocalizer;
import utilities.Util;
import utilities.Clock;
import wifi.WifiConnection;

/**
 * Base robot class with all hardware objects and loaded utilities
 * Handles demo mode selection, initializes objects, starts other threads
 * @version 3.0
 * @author juliette
 *
 */
public class Main {
	//Wifi
	private static WifiConnection conn = null;
	//Resources (motors, sensors)
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final EV3LargeRegulatedMotor forkliftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3MediumRegulatedMotor clawMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("A"));
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
	public static double[] startingCornerCoord = new double[3];
	public static double[][] GREEN;
	public static double[][] RED;
	
	/**
	 * Current action the robot is doing
	 */
	public enum RobotState {Setup, Localization, Search, Capture, Disabled, Avoiding, Returning, Finished};
	/**
	 * 
	 * @author juliette
	 * Select test to run or run in match mode (Default).
	 */
	public enum DemoState {Default, StraightLineTest, SquareTest, LocalizationTest, NavigationTest, SearchTest, ObjectDiffTest, RGBVectorTest, TrackTest, ForkliftTest, Avoidance, GripTest, ReturnTest};	//can be expanded to include alternate options, debugging, hardware tests, etc.
	/**
	 * Robot job. Either builder or garbage collector. Builder is default.
	 */
	public enum RobotTask {Builder, Collector};
	
	public static LCDInfo lcd;
	
	public static final int RESTING_ARM_POSITION = 30;

	/**
	 * Main execution thread.
	 * @param args None used
	 */
	public static void main(String[] args) {
		state = RobotState.Setup;
		Search.searchState = SearchState.Idle;
		
		//Setup threads
		Odometer odo = new Odometer(leftMotor, rightMotor);
		final Navigation nav = new Navigation(odo);
		lcd = new LCDInfo(odo, textLCD, false);	//do not start on creation
		ThreadEnder ender = new ThreadEnder();
		USLocalizer localizer = new USLocalizer(odo, usSensor, Util.US_TO_CENTER);
		forklift = new Forklift(forkliftMotor,clawMotor);
		
		// Default values for these - could be changed by wifi if enabled
		GREEN = new double[][]{{1*Util.SQUARE_LENGTH,1*Util.SQUARE_LENGTH},
			{3*Util.SQUARE_LENGTH,2*Util.SQUARE_LENGTH}};
		RED = new double[][]{{0*Util.SQUARE_LENGTH,5*Util.SQUARE_LENGTH},
				{2*Util.SQUARE_LENGTH,9*Util.SQUARE_LENGTH}};
		startingCorner = 1;
		task = RobotTask.Builder;
		
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
		Capture capture = new Capture(odo, search, GREEN);
		Avoider avoid = new Avoider(odo, nav, usSensor, RED);
		Clock clock = new Clock(odo);
		
		textLCD.clear(); //blank display before selection
		demo = stateSelect();	//select state
		
		//threads intrinsic to all processes
		clock.start();
		odo.start();
		ender.start();
		//lcd.resume();
		
		switch (demo) {
		case Default: //regular robot operation
			lcd.pause();
			textLCD.clear();
			System.out.println("Thin happened");
			gridLineDetector = new LightIntensitySensor(intensityPort);
			/*if(Util.USE_WIFI) {
				HashMap<String, Integer> parameters = null;
				try {
					parameters = wifiConnect();
				} catch (IOException e) {	//failed to connect to wifi
//					Sound.beepSequence();
					System.err.println(e);
					System.exit(-1);
				}
//				Sound.beep();
				if(parameters != null) {
					GREEN = new double[2][2];
					RED = new double[2][2];
					transmissionParse(parameters);
//					Sound.beepSequenceUp();
				}
			} else {	//default parameters
				// Default values for these - could be changed by wifi if enabled
				GREEN = new double[][]{{0*Util.SQUARE_LENGTH,0*Util.SQUARE_LENGTH},
					{3*Util.SQUARE_LENGTH,3*Util.SQUARE_LENGTH}};
				RED = new double[][]{{1*Util.SQUARE_LENGTH,1*Util.SQUARE_LENGTH},
						{2*Util.SQUARE_LENGTH,9*Util.SQUARE_LENGTH}};
				
				startingCorner = 1;
				task = RobotTask.Builder;
				
				StartCorner start = StartCorner.lookupCorner(startingCorner);
				startingCornerCoord[0] = start.getX();
				startingCornerCoord[1] = start.getY();
				startingCornerCoord[2] = Math.PI * (1.0 - (double)startingCorner/2.0);
			}*/
/*			System.out.println("S: "+GREEN[0][0]+" "+GREEN[0][1]);
			System.out.println("A: "+RED[0][0]+" "+RED[0][1]);
			Button.waitForAnyPress(); */
			clock.start();
			System.out.print("\n\n\n\n\n\n\n\n");
			//lcd.resume();
			
			
			state = RobotState.Localization;
			localizer.doLocalization();

			//JUST ADDED TO SPEED UP TESTS, DELETE AFTERWARDS
			//state = RobotState.Search;
			while(state != RobotState.Search){try {
				Thread.sleep(500);	//used to be SLEEP_PERIOD
			} catch (InterruptedException e) {
				e.printStackTrace();
			}}
			/*System.out.println(String.valueOf(odo.getX()));
			System.out.println(String.valueOf(odo.getY()));
			System.out.println(String.valueOf(odo.getTheta()));
			Button.waitForAnyPress();*/
			avoid.start();
			capture.start();
			search.start();
			lcd.resume();
			break;
			
		// Tests need to be verified in this order, 
		// as a test builds on top of the prior one.
		case StraightLineTest:	
			Test.StraightLineTest(odo, Util.SQUARE_LENGTH); // test tachometer/odometer
			break;
		case SquareTest:
			gridLineDetector = new LightIntensitySensor(intensityPort);
			Test.SquareTest(odo, 3, 2 * Util.SQUARE_LENGTH); //test rotation
			break;
		case LocalizationTest:
			Test.LocalizationTest(odo); //test US sensor
			break;
		case NavigationTest:
			// the given points test all major rotation angles: 45, 135, 180, 360. Modify as needed
			Test.NavigationTest(odo, new double[][] {{60, 60}, {60,0}, {30,30}, {60,0}}, true); 
			break;
		case SearchTest:
//			avoid.start();
//			search.start();
//			capture.start();
			state = RobotState.Search;
			break;
		case RGBVectorTest:
			Test.RGBUnitVectorTest(colorSensor);
			break;
		case TrackTest:
			Test.TrackMeasureTest(odo, 10);
			break;
		case ForkliftTest:
			Test.ForkliftTest();
			break;
		case Avoidance:
//			avoid.start();
			Test.AvoidanceTest(odo);
			break;
		case ObjectDiffTest:
			Test.ObjectDiffTest();
			break;
		case GripTest:
			Test.GripTest();
			break;
		case ReturnTest:
			clock.start();
			avoid.start();
			lcd.resume();
			Test.ReturnTest(odo, clock);
			break;
		default:
			System.exit(-1);
		}
		
//		// 5 minute timer
//		Timer timer = new Timer();
//		timer.schedule(new TimerTask() {
//			@Override
//			public void run() {
//				Main.state = RobotState.Finished;
//				Search.searchState = SearchState.Idle;
//				Capture.captureState = CaptureState.Idle;
//				Avoider.avoidState = AvoidState.Enabled;
//				nav.travelTo(startingCornerCoord[0], startingCornerCoord[1]);
//			}	  
//		}, 5*60*1000);
		
		//wait for escape key to end program
		while(Button.waitForAnyPress() != Button.ID_ESCAPE);	
		
//		odo.interrupt();
//		search.interrupt();
//		avoid.interrupt();
//		capture.interrupt();
//		localizer.interrupt();
		
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
	
	/**
	 * Parses transmission information and sets robot parameters.
	 * @param transmission transmission information gotten from wifi.
	 */
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
			
			// store starting corner coordinates
			switch (Main.startingCorner) {
			case 1:
				startingCornerCoord[0] = 0;
				startingCornerCoord[1] = 0;
				startingCornerCoord[2] = (Math.PI/2.0);
					break;
			case 2:
				startingCornerCoord[0] = 10*Util.SQUARE_LENGTH;
				startingCornerCoord[1] = 0;
				startingCornerCoord[2] = Math.PI;
					break;
			case 3:
				startingCornerCoord[0] = 10*Util.SQUARE_LENGTH;
				startingCornerCoord[1] = 10*Util.SQUARE_LENGTH;
				startingCornerCoord[2] = (3.0/2.0)*Math.PI;
					break;
			case 4:
				startingCornerCoord[0] = 0;
				startingCornerCoord[1] = 10*Util.SQUARE_LENGTH;
				startingCornerCoord[2] = 0;
					break;
			}
			
			//Get zone coordinates
			//GREEN is *always* our scoring zone and RED is always our avoiding zone.
			if(task == RobotTask.Builder)
			{
				RED[0][0] = transmission.get("LRZx") * Util.SQUARE_LENGTH;
				RED[0][1] = transmission.get("LRZy") * Util.SQUARE_LENGTH;
				RED[1][0] = transmission.get("URZx") * Util.SQUARE_LENGTH;
				RED[1][1] = transmission.get("URZy") * Util.SQUARE_LENGTH;
				
				GREEN[0][0] = transmission.get("LGZx") * Util.SQUARE_LENGTH;
				GREEN[0][1] = transmission.get("LGZy") * Util.SQUARE_LENGTH;
				GREEN[1][0] = transmission.get("UGZx") * Util.SQUARE_LENGTH;
				GREEN[1][1] = transmission.get("UGZy") * Util.SQUARE_LENGTH;
			} else {
				GREEN[0][0] = transmission.get("LRZx") * Util.SQUARE_LENGTH;
				GREEN[0][1] = transmission.get("LRZy") * Util.SQUARE_LENGTH;
				GREEN[1][0] = transmission.get("URZx") * Util.SQUARE_LENGTH;
				GREEN[1][1] = transmission.get("URZy") * Util.SQUARE_LENGTH;
				
				RED[0][0] = transmission.get("LGZx") * Util.SQUARE_LENGTH;
				RED[0][1] = transmission.get("LGZy") * Util.SQUARE_LENGTH;
				RED[1][0] = transmission.get("UGZx") * Util.SQUARE_LENGTH;
				RED[1][1] = transmission.get("UGZy") * Util.SQUARE_LENGTH;
			}
		}
	}
	
	/**
	 * Connects to wifi and waits for information to be transmitted.
	 * @return Transmission parameters to be parsed
	 * @throws IOException Fails if robot cannot connect to server.
	 * @see Util#IP_ADDR
	 */
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
