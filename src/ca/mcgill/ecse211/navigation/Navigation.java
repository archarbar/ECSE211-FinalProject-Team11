package ca.mcgill.ecse211.navigation;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.odometer.Odometer;

public abstract class Navigation {

  public static GridRectangle safeArea;

  /**
   * this boolean check if the robot's motors are moving and therefore we know if the robot is navigating or not
   * 
   * @return
   */
  public static boolean isNavigating() {
    boolean result = false;
    if (leftMotor.isMoving() && rightMotor.isMoving()) {
      result = true;
    }
    return result;
  }

  /**
   * Converts a distance in cm into angle turned by the wheels in degrees.
   * 
   * @param radius
   * @param distance
   * @return
   */

  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts an angle in degrees into a number of degrees the wheels must turn in order to turn the robot.
   * 
   * @param angle
   * @return
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }

  /**
   * determines if a given (x,y) point (in cm) is safe to travel to.
   * @param x
   * @param y
   * @return
   */
  public static boolean outOfBounds(double x, double y) {
    double width = 0.75 * TRACK;
    return (x > safeArea.getxLow() * TILE_SIZE + width && y > safeArea.getyLow() * TILE_SIZE + width
        && x < safeArea.getxHigh() * TILE_SIZE - width && y < safeArea.getyHigh() * TILE_SIZE - width);
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
   * Turns the robot by a given angle.
   * 
   * @param theta
   */
  public static void turnTo(double theta) {
    leftMotor.rotate(convertAngle(minimumAngle(theta)), true);
    rightMotor.rotate(-convertAngle(minimumAngle(theta)), false);
  }

  /**
   * Turns the robot to a face a given heading.
   * 
   * @param theta
   */
  public static void turnToHeading(double theta) {
    double[] positions = Odometer.getOdometer().getXYT();
    double angle = theta - positions[2];
    angle = minimumAngle(angle);
    turnTo(angle);
  }
  
  public static double calculateDistanceTo(double x, double y) {
    double oX = Odometer.getOdometer().getXYT()[0];
    double oY = Odometer.getOdometer().getXYT()[1];
    double dX = x-oX;
    double dY = y-oY;
    return hyp(dX, dY);
  }
  public static double calculateDistanceTo(int x, int y) {
    double aX = convertGridToLocation(x);
    double aY = convertGridToLocation(y);
    return calculateDistanceTo(aX, aY);
  }
  
  private static double hyp(double dX, double dY) {
    return Math.sqrt(dX*dX+dY*dY);
  }
  public static double convertGridToLocation(int a) {
    return a*TILE_SIZE;
  }
  
  /**
   * Determines the relative angle between heading and target angle.
   * 
   * @return
   */
  public static double angleToTarget(double x, double y) {
    double[] positions = Odometer.getOdometer().getXYT();
    double dx = x - positions[0];
    double dy = y - positions[1];
    double absoluteAngle;

    if (dy == 0 && dx >= 0) {
      absoluteAngle = 90;
    } else if (dy == 0 && dx < 0) {
      absoluteAngle = 180;
    } else {
      absoluteAngle = Math.toDegrees(Math.atan(dx / dy)); // true if y>0
    }
    if (dy < 0) {
      absoluteAngle += 180;
    } else if (dx < 0) {
      absoluteAngle += 360;
    }

    return minimumAngle(absoluteAngle - positions[2]);
  }
  /**
   * Determines the relative angle between heading and target angle.
   * 
   * @return
   */
  public static double angleToTarget(int x, int y) {
    double ax = convertGridToLocation(x);
    double ay = convertGridToLocation(y);
    return angleToTarget(ax, ay);
  }
  
  /**
   * Determines the entrance of a tunnel
   * Takes 4 integers as inputs, which are coordinates of the two tunnel corners
   * Returns the entrance coordinates as an array
   */
  public static int[] findTunnelEntrance(int hole1_x, int hole1_y, int hole2_x, int hole2_y) {
    int[] entrance = {,};
    double currentX = Odometer.getOdometer().getXYT()[0];
    double currentY = Odometer.getOdometer().getXYT()[1];
    boolean xTunnel = false; // set default as false
    if (Math.abs(hole1_x - hole2_x) == 2) { // if x length of tunnel is 2, then it is a horizontal tunnel
      xTunnel = true;
    }
    double dist1 = calculateDistanceTo(hole1_x, hole1_y);
    double dist2 = calculateDistanceTo(hole2_x, hole2_y);
    if (dist1 >= dist2) { // if hole 2 is closer, set it as entrance
      entrance[0] = hole2_x;
      entrance[1] = hole2_y;
    }
    else {                // if hole 1 is closer, set it as entrance
      entrance[0] = hole1_x;
      entrance[1] = hole1_y;
    }
    if (xTunnel) { // if horizontal tunnel
      if (currentX >= entrance[0]) { // if current x position bigger or equal to entrance x position
        entrance[1] -= 1;
      }
      else {                         // if current x position smaller than entrance x position
        entrance[0] -= 1;
      }
    }
    else {         // if vertical tunnel
      if (currentY >= entrance[1]) { // if current y position greater or equal to entrance y position
        entrance[0] -= 1;
      }
      else {                         // if current y position smaller than entrance y position
        entrance[1] -= 1;
      }
    }
    return entrance;
  }
}
