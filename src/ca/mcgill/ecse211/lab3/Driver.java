/*
 * SquareDriver.java
 */
package ca.mcgill.ecse211.lab3;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Driver {
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double TILE_SIZE = 30.48;
	private static double angle = 0; // lab3
	private static double hypo = 0; // lab3
	private static int XTemp = 0;
	private static int YTemp = 0;

	/**
	 * This method is meant to drive the robot in a square of size 2x2 Tiles. It is
	 * to run in parallel with the odometer and Odometer correcton classes allow
	 * testing their functionality.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftRadius
	 * @param rightRadius
	 * @param width
	 */
	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius,
			double rightRadius, double track, int Xaxis, int Yaxis) {

		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}

		XTemp = Xaxis - Lab3.XCor;
		YTemp = Yaxis - Lab3.YCor;

		// drive forward two tiles
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		if (YTemp != 0) {
			angle = Math.atan(XTemp / YTemp); // lab3
		}

		hypo = Math.sqrt((XTemp * XTemp) + (YTemp * YTemp));
		angle = Math.toDegrees(angle);
		angle = Math.abs(angle);

		if (XTemp < 0 && YTemp < 0) {
			angle = angle + 180;
		}

		else if (XTemp > 0 && YTemp < 0) {
			angle = angle + 90;
		}

		else if (XTemp < 0 && YTemp > 0) {
			angle = angle + 270;
		}

		if (XTemp == 0 && YTemp < 0) {
			angle = 180;
		} else if (XTemp == 0 && YTemp > 0) {
			angle = 0;
		} else if (XTemp < 0 && YTemp == 0) {
			angle = 270;
		} else if (XTemp > 0 && YTemp == 0) {
			angle = 90;
		}

		if (angle > 180) {
			angle = angle - 360;
		}

		if (Lab3.ZCor > 0) {
			angle = angle - Lab3.ZCor;
		}

		if (Lab3.ZCor < 0) {
			angle = angle + Lab3.ZCor;
		}

		leftMotor.rotate(convertAngle(leftRadius, track, angle), true); // lab3 rotates
		rightMotor.rotate(-convertAngle(rightRadius, track, angle), false); // lab3

		leftMotor.rotate(convertDistance(leftRadius, hypo * TILE_SIZE), true);
		rightMotor.rotate(convertDistance(rightRadius, hypo * TILE_SIZE), false);

		// updates last co ordinates to use later

		Lab3.XCor = Xaxis;
		Lab3.YCor = Yaxis;
		Lab3.ZCor = Odometer.odo.getXYT()[2];
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
