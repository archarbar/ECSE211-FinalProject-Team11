package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import static ca.mcgill.ecse211.project.Resources.*;
import java.util.LinkedList;

public class Navigation extends Thread {

  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private final double TRACK;
  private final double WHEEL_RAD;
  private Odometer odometer;

  private double[][] targetTile;
  private static final TextLCD lcd = LocalEV3.get().getTextLCD(); //

  int ROTATE_SPEED = 155;

  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, final double TRACK,
      final double WHEEL_RAD, double[][] targetTile) {

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;
    this.odometer = Odometer.getOdometer();

    odometer.setXYT(30.48, 30.48, 0);
    this.targetTile = targetTile;

  }


  double deltaX;
  double deltaY;
  double deltaT;

  double currentX;
  double currentY;
  double currentT;
  double dist;
  double dTheta;

  /**
   * Main entry point.
   */
  public void run() {
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }
    try {
      Thread.sleep(50);
    } catch (InterruptedException e) {

    }



    // find destination to travel to
    double[] destination = findLaunchLocation(Main.getLaunchTarget());
    travelTo(destination[0], destination[1]);
    double[] target = new double[2];
    // find location of target
    target[0] = (Main.getLaunchTarget()[0] + 0.5) * TILE_SIZE;
    target[1] = (Main.getLaunchTarget()[1] + 0.5) * TILE_SIZE;
    turnTo(target[0], target[1]);

  }

  /**
   * the travelTo function takes a waypoint in cm as input. It retrieve the current robot's coordinates using the
   * odometer and calculates the difference of the waypoint's coordinates and the robot coordinates to get the distance
   * it still has to travel to reach the next waypoint. Depending on the deltaX and deltaY coordinate we can know in
   * which quadrant the next waypoint is located.
   * 
   * then using the Math.atan the angle theta ( the direction of where the next waypoint is) will be calculated, it will
   * then be substracted from the current robot's heading and we will therefore know the correction we have to apply to
   * our robot's heading in order to reach the next waypoint Finally the turnTo() method is called to get the minimal
   * angle correction.
   * 
   * @param x
   * @param y
   */


  /**
   * Moves robot to (x,y) location.
   * 
   * @param x
   * @param y
   */
  void travelTo(double x, double y) {
    turnTo(x, y);


    leftMotor.setSpeed(180);
    rightMotor.setSpeed(180);
    leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, dist), false);


  }


  /**
   * this method takes an angle and returns the minimum angle needed to turn to reach the next waypoint. ex: if
   * thetaDiff indicates that the robot should turn 240 degree in a certain direction. It is the same as turning 120
   * degree in the other direction. We therefore, minimize the angle
   * 
   * @param theta
   */
  void turnTo(double theta) {

    if (theta > 180) {
      // minimum angle: turning 240 in one direction is the same as turning 120 degrees in the other

      theta = 360 - theta;
      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);

    }

    else if (theta < -180) {
      theta = 360 + theta;
      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);

    }

    else {
      leftMotor.setSpeed(ROTATE_SPEED);
      leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      rightMotor.setSpeed(ROTATE_SPEED);
      rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);

    }
  }

  /**
   * Turns the robot to face a position given x, and y in cm.
   * 
   * @param x
   * @param y
   */
  void turnTo(double x, double y) {
    double position[] = new double[3];
    position = odometer.getXYT();
    currentX = position[0];
    currentY = position[1];
    currentT = position[2];

    deltaX = x - currentX;
    deltaY = y - currentY;
    dist = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    if (deltaY >= 0) {
      // quadrant 1 and 2

      dTheta = Math.atan(deltaX / deltaY);


    } else if (deltaY <= 0 && deltaX >= 0) {

      // quadrant 4

      // this is done because the range of arctan is between -pi/2 and pi/2 so we have to factor in Math.PI

      dTheta = Math.atan(deltaX / deltaY) + Math.PI;

    } else {

      // quadrant 3

      dTheta = Math.atan(deltaX / deltaY) - Math.PI;
    }

    double thetaDiff = (Math.toDegrees(dTheta) - currentT);

    turnTo(thetaDiff);
  }

  /**
   * this boolean check if the robot's motors are moving and therefore we know if the robot is navigating or not
   * 
   * @return
   */
  boolean isNavigating() {

    boolean result = false;
    if (leftMotor.isMoving() && rightMotor.isMoving()) {
      result = true;
    }

    return result;
  }

  /**
   * convert distance and convert angles were both methods taken from the the SquareDriver class they convert distance
   * or angle respectively into the appropriate number of tacho counts the motor has to perform.
   * 
   * 
   * @param radius
   * @param distance
   * @return
   */

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }


  /**
   * Finds angle to rotate wheels by to rotate by in order to turn <angle> degrees.
   * 
   * @param radius
   * @param width
   * @param angle
   * @return
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * determines if a location is completely outside of the grid.
   * 
   * @param x
   * @param y
   * @return
   */
  public static boolean outOfBounds(double x, double y) {
    return (x > 0 && y > 0 && x < numTilesX * TILE_SIZE && y < numTilesY * TILE_SIZE);
  }


  /**
   * calculates the equivalent angle that is closest to 0.
   * 
   * @param theta
   * @return
   */
  public static double minimumAngle(double theta) {
    while (Math.abs(theta) > 180) {
      while (theta > 180) {
        theta -= 360;
      }
      while (theta < -180) {
        theta += 360;
      }
    }
    return theta;
  }

  /**
   * Turns the robot to a relative angle.
   * 
   * @param theta
   */
  public static void turn(double theta) {
    Resources.leftMotor.rotate(convertAngle(minimumAngle(theta)), true);
    Resources.rightMotor.rotate(-convertAngle(minimumAngle(theta)), false);
  }

  /**
   * Turns the robot to an absolute orientation angle.
   * 
   * @param theta
   */
  public static void absTurnTo(double theta) {
    double[] positions = Odometer.getOdometer().getXYT();
    double angle = theta - positions[2];
    angle = minimumAngle(angle);
    turn(angle);
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that angle.
   *
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * Resources.TRACK * angle / 360.0);
  }

  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   *
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * Resources.WHEEL_RAD));
  }

  /**
   * Finds the valid launch location along the diagonal and return it. Each target location will have 1 valid diagonal
   * launch location.
   * 
   * @param targetLocation
   * @return
   */
  public static double[] findLaunchLocation(int targetLocation[]) {
    boolean spaceX = false, spaceY = false, spacex = false, spacey = false;
    if (targetLocation[0] + 4 <= 7) {
      spaceX = true;
    }
    if (targetLocation[0] - 4 >= 0) {
      spacex = true;
    }
    if (targetLocation[1] + 4 <= 7) {
      spaceY = true;
    }
    if (targetLocation[1] - 4 >= 0) {
      spacey = true;
    }

    double[] destination = new double[2];
    if (spacex && spacey) {
      destination[0] = (targetLocation[0] - 3) * TILE_SIZE;
      destination[1] = (targetLocation[1] - 3) * TILE_SIZE;
    }
    if (spaceX && spacey) {
      destination[0] = (targetLocation[0] + 4) * TILE_SIZE;
      destination[1] = (targetLocation[1] - 3) * TILE_SIZE;
    }
    if (spacex && spaceY) {
      destination[0] = (targetLocation[0] - 3) * TILE_SIZE;
      destination[1] = (targetLocation[1] + 4) * TILE_SIZE;
    }
    if (spaceX && spaceY) {
      destination[0] = (targetLocation[0] + 4) * TILE_SIZE;
      destination[1] = (targetLocation[1] + 4) * TILE_SIZE;
    }
    return destination;
  }

  /**
   * Calculates euclidean distance to a location.
   * 
   * @param location
   * @return
   */
  private static double calculateDistance(double[] location) {
    double x = location[0] - TILE_SIZE;
    double y = location[1] - TILE_SIZE;
    return Math.sqrt(x * x + y * y);
  }

  /**
   * Determines if a given location to travel to is within the grid.
   * 
   * @param targetLocation
   * @return
   */
  private static boolean isValidLaunchLocation(double targetLocation[]) {
    return targetLocation[0] >= TILE_SIZE && targetLocation[1] >= TILE_SIZE
        && targetLocation[0] <= (numTilesX - 1) * TILE_SIZE && targetLocation[1] <= (numTilesY - 1) * TILE_SIZE;
  }

  /**
   * Converts from grid location to location in cm.
   * 
   * @param waypoint
   * @return
   */
  public static double[] waypointToLocation(int[] waypoint) {
    double[] location = new double[waypoint.length];
    for (int i = 0; i < waypoint.length; i++) {
      location[i] = waypoint[i] * TILE_SIZE + TILE_SIZE / 2;
    }
    return location;
  }
}
