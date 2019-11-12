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
   *
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
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
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

  public static void moveTo(double distance) {
    leftMotor.setSpeed(MOTOR_SPEED);
    rightMotor.setSpeed(MOTOR_SPEED);
    int angle = convertDistance(distance);
    leftMotor.rotate(angle, true);
    rightMotor.rotate(angle, false);
  }

  public static double calculateDistanceTo(double x, double y) {
    double oX = Odometer.getOdometer().getXYT()[0];
    double oY = Odometer.getOdometer().getXYT()[1];
    double dX = x - oX;
    double dY = y - oY;
    return hyp(dX, dY);
  }

  public static double calculateDistanceTo(int x, int y) {
    double aX = convertGridToLocation(x);
    double aY = convertGridToLocation(y);
    return calculateDistanceTo(aX, aY);
  }

  private static double hyp(double dX, double dY) {
    return Math.sqrt(dX * dX + dY * dY);
  }

  public static double convertGridToLocation(int a) {
    return a * TILE_SIZE;
  }

  public static int convertLocationToGrid(double a) {
    return (int) Math.round(a / TILE_SIZE);
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
   * Determines the entrance of a tunnel Takes 4 integers as inputs, which are coordinates of the two tunnel corners
   * Returns the entrance coordinates as an array
   */
  public static Point findTunnelEntrance(Point hole1, Point hole2) {
    Point entrance;
    Point currentPos = new Point(Odometer.getOdometer().getXYT()[0], Odometer.getOdometer().getXYT()[1]);
    boolean xTunnel = false; // set default as false
    if ((int) Math.abs((hole1.x - hole2.x) / TILE_SIZE) == 2) { // if x length of tunnel is 2, then it is a
                                                                // horizontal tunnel
      xTunnel = true;
    }
    double dist1 = calculateDistanceTo(hole1.x, hole1.y);
    double dist2 = calculateDistanceTo(hole2.x, hole2.y);
    if (dist1 >= dist2) { // if hole 2 is closer, set it as entrance
      entrance = new Point(hole2);
    } else { // if hole 1 is closer, set it as entrance
      entrance = new Point(hole1);
    }
    if (xTunnel) { // if horizontal tunnel
      if (currentPos.x >= entrance.x) { // if current x position bigger or equal to entrance x position
        entrance.y -= TILE_SIZE;
      } else { // if current x position smaller than entrance x position
        entrance.x -= TILE_SIZE;
      }
    } else { // if vertical tunnel
      if (currentPos.y >= entrance.y) { // if current y position greater or equal to entrance y position
        entrance.x -= TILE_SIZE;
      } else { // if current y position smaller than entrance y position
        entrance.y -= TILE_SIZE;
      }
    }
    entrance.x += TILE_SIZE / 2;
    entrance.y += TILE_SIZE / 2;
    return entrance;
  }

  /**
   * Finds the point in the launching zone that is closest to the bin Takes 2 inputs: the point representing the
   * coordinates of the bin, and a GridRectagle for the launch zone returns a point, which is the closest launch point
   * in the zone
   */
  public static Point findClosestLaunchPoint(Point binLocation, GridRectangle launchZone) {
    Point location = new Point(0, 0);
    if (binLocation.x <= launchZone.getxLow()) {
      location.x = launchZone.getxLow() + TILE_SIZE / 2;
    } else if (binLocation.x >= launchZone.getxHigh()) {
      location.x = launchZone.getxHigh() - TILE_SIZE / 2;
    } else {
      location.x = binLocation.x;
    }
    if (binLocation.y <= launchZone.getyLow()) {
      location.y = launchZone.getyLow() + TILE_SIZE / 2;
    } else if (binLocation.y >= launchZone.getyHigh()) {
      location.y = launchZone.getyHigh() - TILE_SIZE / 2;
    } else {
      location.y = binLocation.y;
    }
    return location;
  }

  /*
   * Finds the point in the launching zone that is at a given distance to the bin Takes 3 inputs: the point representing
   * the coordinates of the bin, a GridRectagle for the launch zone, and the desired distance in cm First, checks if the
   * point on the circle closest to current position is in the launch zone. If it is, return it, if not, then find the
   * closest point on the circle to current position that is inside the launch zone. This second point will be part of
   * the list of edge angle values for the circle. Returns a point, which is the launch point in the zone with the given
   * distance to the bin, within a margin of error of 0.5 cm.
   */
  public static Point findLaunchPoint(Point bin, GridRectangle launchZone, double distance) {
    double[] t = {0, 0, 0, 0, 0, 0, 0, 0};
    double minDist = -1;
    double newDist;
    double minAngle = 0;
    Point closestPoint = new Point(0, 0); // closest point on circle to current location
    Point location = new Point(0, 0); // final launch location at given distance
    Point currentLocation = new Point(0, 0); // current location of robot
    double[] position = Odometer.getOdometer().getXYT();
    currentLocation.x = position[0];
    currentLocation.y = position[1];
    closestPoint.x = bin.x + distance * ((currentLocation.x - bin.x) // formula to find the point on a circle
                                                                     // that is closest to another point
        / Math.sqrt(Math.pow(currentLocation.x - bin.x, 2) + Math.pow(currentLocation.y - bin.y, 2)));
    closestPoint.y = bin.y + distance * ((currentLocation.y - bin.y) // formula to find the point on a circle
                                                                     // that is closest to another point
        / Math.sqrt(Math.pow(currentLocation.x - bin.x, 2) + Math.pow(currentLocation.y - bin.y, 2)));
    if (launchZone.contains(closestPoint)) {
      return closestPoint;
    } else {
      t[0] = Math.acos(launchZone.getxLow() / distance);
      t[1] = -1 * t[0];
      t[2] = Math.acos(launchZone.getxHigh() / distance);
      t[3] = -1 * t[3];
      t[4] = Math.asin(launchZone.getyLow() / distance);
      t[5] = Math.PI - t[4];
      t[6] = Math.asin(launchZone.getyHigh() / distance);
      t[7] = Math.PI - t[6];
      for (double angle : t) {
        Point edgePoint = new Point(distance * Math.cos(angle), distance * Math.sin(angle));
        if (launchZone.contains(edgePoint)) {
          newDist = calculateDistanceTo(edgePoint.x, edgePoint.y);
          if (minDist == -1) {
            minDist = newDist;
            minAngle = angle;
          }
          else if (newDist < minDist) {
            minDist = newDist;
            minAngle = angle;
          }
        }
      }
      location.x = distance * Math.cos(minAngle);
      location.y = distance * Math.sin(minAngle);
      return location;
    }
  }

  public static void forwards() {
    leftMotor.forward();
    rightMotor.forward();
  }

  public static void stop() {
    leftMotor.stop();
    rightMotor.stop();
  }

  public void travelTo(int x, int y, boolean centre) {
    double aX = x;
    double aY = y;
    if (centre) {
      aX += 0.5;
      aY += 0.5;
    }
    travelTo(aX * TILE_SIZE, aY * TILE_SIZE);
  }

  public abstract void travelTo(double x, double y);

}
