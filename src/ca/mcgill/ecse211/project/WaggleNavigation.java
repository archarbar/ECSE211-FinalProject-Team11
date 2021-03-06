package ca.mcgill.ecse211.project;

import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * A navigator that actively corrects its heading and position.
 * Named as such t=due to waggling as it corrects its heading and position.
 *
 * @author Matthew
 *
 */
public class WaggleNavigation extends Navigation {
  enum Axis {
    X, Y
  };
  enum Direction {
    POS, NEG
  }

  Odometer odometer = Odometer.getOdometer();

  /**
   * left light sensor data
   */
  float[] csDataL;

  /**
   * right light sensor data
   */
  float[] csDataR;

  /**
   * distance between starting and current positions
   */
  private double distance;

  /**
   * initial position of robot from odometer
   */
  double[] startPos;

  /**
   * final position of robot from odometer
   */
  double[] endPos;

  /**
   * difference in theta between starting and current angles
   */
  double offTheta;

  /**
   * boolean to know which side robot is facing
   */
  boolean isLeft = false;

  /**
   * red light intensity to compare read colors
   */
  static final float LINE_RED_INTENSITY = 0.35f; // cast to float since default is double

  /**
   * current position from odometer
   */
  double[] position;

  /**
   * our line detector controller for left light sensor
   */
  private LineDetectorController detectorL; // use a red light to compare luminence level

  /**
   * our line detector controller for right light sensor
   */
  private LineDetectorController detectorR;

  public WaggleNavigation() {

  }

  /**
   * Initializes the navigator with 2 LineDetectors.
   * @param l the left line detector.
   * @param r the right line detector.
   */
  public WaggleNavigation(LineDetectorController l, LineDetectorController r) {
    detectorL = l;
    detectorR = r;
  }

  /**
   * Tells the robot to travel from its current location to a given point in cm travelling parrallel to the gridlines
   * and while actively correcting.
   *
   * @param x in cm
   * @param y in cm
   */
  @Override
  public void travelTo(double x, double y) {
    double heading, heading2;
    Direction dir1, dir2;
    Axis axis1, axis2;
    int tileDis1, tileDis2;
    position = Odometer.getOdometer().getXYT();
    double dX = (x - position[0]);
    double dY = (y - position[1]);
    int xTileDis = (int) Math.floor(Math.abs(dX / TILE_SIZE));
    int yTileDis = (int) Math.floor(Math.abs(dY / TILE_SIZE));
    if (xTileDis > yTileDis) {
      axis1 = Axis.X;
      axis2 = Axis.Y;
      tileDis1 = xTileDis;
      tileDis2 = yTileDis;
      if (dX < 0) {
        dir1 = Direction.NEG;
        heading = 270;
      } else {
        dir1 = Direction.POS;
        heading = 90;
      }
      if (dY < 0) {
        dir2 = Direction.NEG;
        heading2 = 180;
      } else {
        dir2 = Direction.POS;
        heading2 = 0;
      }
    } else {
      axis1 = Axis.Y;
      axis2 = Axis.X;
      tileDis1 = yTileDis;
      tileDis2 = xTileDis;
      if (dY < 0) {
        dir1 = Direction.NEG;
        heading = 180;
      } else {
        dir1 = Direction.POS;
        heading = 0;
      }
      if (dX < 0) {
        dir2 = Direction.NEG;
        heading2 = 270;
      } else {
        dir2 = Direction.POS;
        heading2 = 90;
      }
    }
    System.out.println("X:" + xTileDis);
    System.out.println("Y:" + yTileDis);
    System.out.println("T:" + angleToTarget(x, y));
    System.out.println("T2:" + heading);



    // turn to heading
    turnToHeading(heading);
    toWaggle(dir1, axis1, tileDis1);
    if (avoided) {
      return;
    }
    Main.sleep();
    turnToHeading(heading2);
    toWaggle(dir2, axis2, tileDis2);
    if (avoided) {
      return;
    }
    Main.sleep();
    position = odometer.getXYT();
    System.out.println("(" + position[0] + "," + position[1] + ")");
    System.out.println("(" + x + "," + y + ")");
    heading = angleToTarget(x, y);
    System.out.println(heading + "*");
    turnTo(heading);
    double distance = calculateDistanceTo(x, y);
    System.out.println(distance + " cm");
    moveTo(distance);
    if (avoided) {
      return;
    }
  }

  /**
   * sets up the waggle method to happen a given amount of times.
   *
   * @param direction that the robot is facing
   * @param axis that the robot is aligned to
   * @param tileDis to travel
   */ 
  private void toWaggle(Direction direction, Axis axis, int tileDis) {
    for (int i = 0; i < tileDis; ++i) {
      if (avoided) {
        return;
      }
      System.out.println("D:" + (tileDis - i));
      waggle(axis, direction);
    }
  }

  /**
   * sets up the main waggle method.
   *
   * @param a is the axis that the robot is aligned to
   * @param d is the direction that the robot is facing
   */
  void waggle(Axis a, Direction d) {
    int axis = 0, direction = 0;
    if (a == Axis.X) {
      axis = 0;
    } else if (a == Axis.Y) {
      axis = 1;
    }
    if (d == Direction.POS) {
      direction = 1;
    } else if (d == Direction.NEG) {
      direction = -1;
    }
    if(detectorL==null || detectorR==null) {
      waggle(axis, direction);
    } else {
      waggle(axis, direction, detectorL, detectorR);
    }
  }

  /**
   * moves forwards until a line is detected, then uses the difference in location to detecting the the same line with
   * the other sensor to correct the angle, it also corrects its position.
   *
   * @param direction 0 for x, 1 for y
   * @param side 1 for positive, -1 for negative
   */
  public synchronized void waggle(int direction, int side) {
    /**
     * left light sensor sample provider
     */
    SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level

    /**
     * right light sensor sample provider
     */
    SampleProvider colorSampleProviderR = colorSensorR.getRedMode();

    /**
     * Initializes the navigator with 2 colour sensors.
     */
    csDataL = new float[colorSensorL.sampleSize()];
    csDataR = new float[colorSensorL.sampleSize()];
    long correctionStart, correctionEnd, deltaCorrection;
    stop();
    forwards();
    while (true) {
      correctionStart = System.currentTimeMillis();
      if (avoided) {
        try {
          wait();
          return;
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        leftMotor.stop(true);
        rightMotor.stop(false);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      
       deltaCorrection = correctionEnd - correctionStart;
      if (deltaCorrection < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    forwards();
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (avoided) {
          try {
            wait();
            return;
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        if (tooFar(startPos)) {
          waggle(direction, side);
          return;
        }
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(-offTheta);
          break;
        }
        correctionEnd = System.currentTimeMillis();
         deltaCorrection = correctionEnd - correctionStart;
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (avoided) {
          try {
            wait();
            return;
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        if (tooFar(startPos)) {
          waggle(direction, side);
          return;
        }
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(offTheta);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd - correctionStart;
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
    position = odometer.getXYT();
    moveTo(sensorOffset);
    if (direction == 0) {
      odometer.setX(roundToLine(position[0]));
    } else if (direction == 1) {
      odometer.setY(roundToLine(position[1]));
    }
  }

  /**
   * moves forwards until a line is detected, then uses the difference in location to detecting the the same line with
   * the other sensor to correct the angle, it also corrects its position.
   *
   * @param direction 0 for x, 1 for y
   * @param side 1 for positive, -1 for negative
   * @param detectorL left line detector
   * @param detectorR right line detector
   */
  private void waggle(int direction, int side, LineDetectorController detectorL, LineDetectorController detectorR) {
    long correctionStart, correctionEnd,deltaCorrection;
  
    stop();
    forwards();
    while (true) {
      
      correctionStart = System.currentTimeMillis();
      boolean detectedLeft = detectorL.lineDetected(), detectedRight = detectorR.lineDetected();
      if ((detectedLeft) || (detectedRight)) {
        stop();
        if (detectedRight) {
          isLeft = false;
        } else if (detectedLeft) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      
       deltaCorrection = correctionEnd - correctionStart; 
      
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    forwards();
    
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (tooFar(startPos)) {
          waggle(direction, side);
          return;
        }
        if (detectorR.lineDetected()) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(-offTheta);
          break;
        }
        correctionEnd = System.currentTimeMillis();
         deltaCorrection = correctionEnd - correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (tooFar(startPos)) {
          waggle(direction, side);
          return;
        }
        if (detectorL.lineDetected()) {
          leftMotor.stop(true);
          rightMotor.stop(false);
          endPos = odometer.getXYT();
          distance = endPos[direction] - startPos[direction];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(offTheta);
          break;
        }
        correctionEnd = System.currentTimeMillis();
         deltaCorrection = correctionEnd - correctionStart;
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (deltaCorrection));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
    position = odometer.getXYT();
    moveTo(sensorOffset);
    if (direction == 0) {
      odometer.setX(roundToLine(position[0]));
    } else if (direction == 1) {
      odometer.setY(roundToLine(position[1]));
    }
  }
  /**
   * given a position on an axis, return the nearest value that's on a gridline.
   *
   * @param pos is the position on an axis
   * @return nearest value that's on a gridline
   */
  private static double roundToLine(double pos) {
    double round = Math.round(pos / TILE_SIZE) * TILE_SIZE;
    return round;
  }

  /**
   * returns true if its been too far since the last line detected for it to be the same line.
   *
   * @param startPos is a given starting location.
   * @return if it has traveled too far for it to be the same line.
   */
  private static boolean tooFar(double[] startPos) {
    double position[] = Odometer.getOdometer().getXYT();
    double dX = startPos[0] - position[0];
    double dY = startPos[1] - position[1];
    double dis = hyp(dX, dY);
    return dis > 5;
  }
}
