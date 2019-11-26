package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Main.*;

/**
 * Contains all the general navigation related methods. Objects of this type also must implement a travelTo method that
 * takes the robot to a specific location.
 *
 * @author Matthew
 *
 */
public abstract class Navigation {

  public static GridRectangle safeArea;

  public boolean avoided = false;

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
//    if(colorSensorR!=null||colorSensorR!=null) {
//      colorSensorL.close();
//      colorSensorR.close();
//    }
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.rotate(convertAngle(minimumAngle(theta)), true);
    rightMotor.rotate(-convertAngle(minimumAngle(theta)), false);// 1.0005
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

  /**
   * Moves the robot in a straight line by a distance in cm.
   *
   * @param distance to travel in cm.
   */
  public static void moveTo(double distance) {
    leftMotor.setSpeed(MOTOR_SPEED);
    rightMotor.setSpeed(MOTOR_SPEED);
    int angle = convertDistance(distance);
    leftMotor.rotate(angle, true);
    rightMotor.rotate(angle, false);
  }

  /**
   * Calculates the distance from the robot's current position and the given location in cm.
   *
   * @param x is the x location in cm.
   * @param y is the y location in cm.
   * @return the distance to this location in cm.
   */
  public static double calculateDistanceTo(double x, double y) {
    double oX = Odometer.getOdometer().getXYT()[0];
    double oY = Odometer.getOdometer().getXYT()[1];
    double dX = x - oX;
    double dY = y - oY;
    return hyp(dX, dY);
  }

  /**
   * Calculates the distance from the robot's current position and the given location in cm.
   *
   * @param x is the x grid location.
   * @param y is the y grid location.
   * @return the distance to this location in cm.
   */
  public static double calculateDistanceTo(int x, int y) {
    double aX = convertGridToLocation(x);
    double aY = convertGridToLocation(y);
    return calculateDistanceTo(aX, aY);
  }

  /**
   * Calculates the length of the hypotenuse of a triangle with sides x, and y.
   *
   * @param dX length of side x.
   * @param dY length of side y.
   * @return the length of the hypotenuse.
   */
  protected static double hyp(double dX, double dY) {
    return Math.sqrt(dX * dX + dY * dY);
  }

  /**
   * Converts a gridlocation integer into a location double in cm.
   *
   * @param a
   * @return
   */
  public static double convertGridToLocation(int a) {
    return a * TILE_SIZE;
  }

  /**
   * Converts a location double in cm into a gridlocation integer.
   *
   * @param a
   * @return
   */
  public static int convertLocationToGrid(double a) {
    return (int) Math.round(a / TILE_SIZE);
  }

  /**
   * Determines the relative angle between heading and target angle.
   *
   *@param double x, x coordinate of target in cm
   *@param double y, y coordinate of target in cm
   * @return minimum angle to target
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
   * @param int x, x coordinate of target, in tiles
   * @param int y, y coordinate of target, in tiles
   * @return minimum angle to target
   */
  public static double angleToTarget(int x, int y) {
    double ax = convertGridToLocation(x);
    double ay = convertGridToLocation(y);
    return angleToTarget(ax, ay);
  }

  /**
   * Determines the entrance of a tunnel.
   * Takes 2 Points as inputs, the two tunnel corners
   * Returns the entrance coordinates as an array
   *
   * @param Point 1, first tunnel corner
   * @param Point 2, second tunnel corner
   * @return entrance of tunnel relative to current position
   */
  public static Point findTunnelEntrance(Point hole1, Point hole2) {
    Point entrance;
    Point currentPos = new Point(Odometer.getOdometer().getXYT()[0], Odometer.getOdometer().getXYT()[1]);
    boolean xTunnel = false; // set default as false
    if ((int) Math.abs(Math.round((hole1.x - hole2.x) / TILE_SIZE)) == 2) { // if x length of tunnel is 2, then it is a
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
  
  /**
   * Takes all the distance from valid distances and finds the points corresponding them with findLaunchPoint. 
   * Then check each point and return the one closest to current position.
   * 
   * @return point closest to current position from all points at valid distances
   */
  public static Point findBestLaunchPoint() {
    double minDist = -1;
    double newDist;
    Point newP = new Point(0, 0);
    Point bestP = new Point(0, 0);
    for (double dist: validDistances) {
      newP = findLaunchPoint(BIN, islandRectangle, dist);
      newDist = calculateDistanceTo(newP.x, newP.y);
      if ((minDist == -1) || (newDist < minDist)) {
        minDist = newDist;
        bestP = newP;
        launchDist = dist;
      }
      else {
        continue;
      }
    }
    return bestP;
  }
    
  
  /**
   * Finds the point in the launching zone that is at a given distance to the bin. Takes 3 inputs: the point representing
   * the coordinates of the bin, a GridRectagle for the launch zone, and the desired distance in cm First, checks if the
   * point on the circle closest to current position is in the launch zone. If it is, return it, if not, then find the
   * closest point on the circle to current position that is inside the launch zone. This second point will be part of
   * the list of edge angle values for the circle. Returns a point, which is the launch point in the zone with the given
   * distance to the bin, within a margin of error of 0.5 cm.
   *
   * @param Point bin location
   * @param GridRectangle for launch zone
   * @param double distance, wanted launch distance
   * @return Point launch point at wanted distance from bin
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
          if ((minDist == -1) || (newDist < minDist)) {
            minDist = newDist;
            minAngle = angle;
          }
          else {
            continue;
          }
        }
      }
      location.x = distance * Math.cos(minAngle);
      location.y = distance * Math.sin(minAngle);
      return location;
    }
  }

  /**
   * Tells the robot to move forwards until otherwise directed.
   */
  public static void forwards() {
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }

  /**
   * Tells the robot to stop moving.
   */
  public static void stop() {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0);
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.resetTachoCount();
    rightMotor.resetTachoCount();
  }



  /**
   * Tells the robot to travel from its current location to the centre of the given tile.
   *
   * @param tile's x value.
   * @param tile's y value.
   */
  public void travelTo(int x, int y, boolean centre) {
    double aX = x;
    double aY = y;
    if (centre) {
      aX += 0.5;
      aY += 0.5;
    }
    travelTo(aX * TILE_SIZE, aY * TILE_SIZE);
  }

  /**
   * Tells the robot to travel from its current location to a given point in cm.
   *
   * @param x in cm
   * @param y in cm
   */
  public abstract void travelTo(double x, double y);

  public synchronized void avoidanceover() {
    avoided = false;
    notifyAll();
  }

  /**
   * Waits for thread to pause before continuing.
   */
  public synchronized void avoidance() {
    return;
  }

}
