package ca.mcgill.ecse211.lab3;

public class Test {

	public static void main(String[] args) {
		
	int x, y;
	
	x=1;
	y=2;
	
	int oX=0;
	int oY=0;
	
	
	/**
	 * package ca.mcgill.ecse211.lab3;


import lejos.hardware.Button;
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
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 15.75;
  public static final int X = 1;
  public static final int Y = 1;


  //public static Port portColor = LocalEV3.get().getPort("S1"); //SK SENSOR
 // public static SensorModes myColor = new EV3ColorSensor(portColor); //SK SENSOR
 // public static SampleProvider myColorSample = myColor.getMode("Red"); //sk sensor                                                      
 // public static float[] sampleColor = new float[myColor.sampleSize()]; //sk sensor
 // public static int lineCounter =0;
 // public static int yCorrection=0;
  
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
    OdometryCorrection odometryCorrection = new OdometryCorrection(); // TODO Complete
                                                                      // implementation
    Display odometryDisplay = new Display(lcd); // No need to change
    
     
    do {
      // clear the display
      lcd.clear();

      // ask the user whether the motors should drive in a square or float
      lcd.drawString("Press Right ", 0, 0);
      lcd.drawString("to Start", 0, 1);

      

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

     if (buttonChoice == Button.ID_RIGHT) {
      // clear the display
      lcd.clear();

      // Start odometer and display threads
      Thread odoThread = new Thread(odometer);
      odoThread.start();
      Thread odoDisplayThread = new Thread(odometryDisplay);
      odoDisplayThread.start();

      // Start correction if right button was pressed
      
        Thread odoCorrectionThread = new Thread(odometryCorrection);
        odoCorrectionThread.start();
      

      // spawn a new Thread to avoid Driver.drive() from blocking
      (new Thread() {
        public void run() {
          Driver.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK, X, Y);
        }
      }).start();
    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}

	 */
		
		
	}
}
