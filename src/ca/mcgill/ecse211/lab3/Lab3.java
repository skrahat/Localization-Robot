package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.UltrasonicLocalizer;

import ca.mcgill.ecse211.lab3.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Motor Objects, and Robot related parameters
	// Sensor port
	public static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 15.75;
	public static int XCor = 0;
	public static int YCor = 0;
	public static double ZCor = 0;

	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 75;
	private static final double TILE_SIZE = 30.48;
	private static double angle = 0; // lab3
	private static double hypo = 0; // lab3
	private static double temp = 0; // lab3
	private static int act; // lab3

	  


	@SuppressWarnings("resource")
	public static SampleProvider usDistance = new EV3UltrasonicSensor(usPort).getMode("Distance");
	public static float[] usData = new float[usDistance.sampleSize()];

	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;
		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		// implementation
		Display odometryDisplay = new Display(lcd);
		UltrasonicLocalizer u = new UltrasonicLocalizer();

		do {
			// clears the display
			lcd.clear();

			// Screen display
			lcd.drawString("Press Right ", 0, 0);
			lcd.drawString("for Rising Edge", 0, 1);
			lcd.drawString("Press Left", 0, 2);
			lcd.drawString("for Falling Edge", 0, 3);
			buttonChoice = Button.waitForAnyPress(); // Records choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		// Rising edge
		if (buttonChoice == Button.ID_RIGHT) {
			// clear the display
			lcd.clear();

			// this instance
			// returned
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			

			// spawn a new Thread to avoid Driver.drive() from blocking
				/*
				 * This is our run method. We called the methods here.
				 * 
				 * @see java.lang.Thread#run()
				 */

					act=1;	
					UltrasonicLocalizer.UltrasonicLocalizerMain(act);
					// risingEdge();

				

		}
		// Falling Edge
		if (buttonChoice == Button.ID_LEFT) {
			// clear the display
			lcd.clear();

			// this instance
			// returned
			// Start odometer and display threads
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			// spawn a new Thread to avoid Driver.drive() from blocking
				/*
				 * This is our run method. We called the methods here.
				 * 
				 * @see java.lang.Thread#run()
				 */

					act=2;	
					UltrasonicLocalizer.UltrasonicLocalizerMain(act);

		

		}
		
			// clears the display
			lcd.clear();

			// Screen display
	
			
			buttonChoice = Button.waitForAnyPress(); // Records choice (left or right press)
		
		if (buttonChoice == Button.ID_RIGHT) {
			lcd.clear();

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();

		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
////////////////////////////
