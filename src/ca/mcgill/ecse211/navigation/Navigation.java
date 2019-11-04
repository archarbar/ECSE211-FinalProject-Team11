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

}