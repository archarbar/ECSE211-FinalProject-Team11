package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.*;

/**
 * localises the robot using colour sensors to detect lines arranged in a grid in order to localise angle and position.
 * 
 * @author Matthew
 * @author Victor
 */
public class ModifiedLightLocalizer extends PlainNavigation {
  
  /**
   * distance between starting and current positions
   */
  private double distance;
  
  /**
   * initial position of robot from odometer
   */
  private double[] startPos;
  
  /**
   * our odometer
   */
  private Odometer odometer;
  
  /**
   * difference in theta between starting and current angles
   */
  private double offTheta;
  
  /**
   * boolean to know which side robot is facing
   */
  private boolean isLeft = false;
  
  /**
   * our line detector controller for left light sensor
   */
  private LineDetectorController detectorL; // use a red light to compare luminence level
  
  /**
   * our line detector controller for right light sensor
   */
  private LineDetectorController detectorR;

  /**
   * initialises 2 light sensors and gets the odometer.
   * @param detectorL is the left line detector
   * @param detectorR is the right line detector
   */
  public ModifiedLightLocalizer(LineDetectorController detectorL, LineDetectorController detectorR) {
    this.odometer = Odometer.getOdometer();
    this.detectorL = detectorL;
    this.detectorR = detectorR;
  }

  /**
   * tells the robot to localise and set the given point as its given location.
   * 
   * @param x position to localize to.
   * @param y position to localize to.
   */
  public void localize(int x, int y) {
    findDistance(x, y);
  }

  /**
   * Localizes the robot then sets its final position to the given values.
   * 
   * @param x position to localize to.
   * @param y position to localize to.
   */
  private void findDistance(int x, int y) {
    odometer.setXYT(0, 0, 0);
    long correctionStart, correctionEnd, deltaCorrection;
    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    stop();
    forwards();

    while (true) {
      correctionStart = System.currentTimeMillis();
      boolean detectedLeft = detectorL.lineDetected(), detectedRight = detectorR.lineDetected();
      if ((detectedLeft) || (detectedRight)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        stop();
        if (detectedRight) {
          isLeft = false;
        } else if (detectedLeft) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        // new Thread(odometer).start();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      deltaCorrection = correctionEnd - correctionStart;
      if (deltaCorrection < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    forwards();
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (detectorR.lineDetected()) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(-offTheta);
          moveTo(-REVERSE_DIST);
          turnTo(90);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd - correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (detectorL.lineDetected()) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(offTheta);
          moveTo(-REVERSE_DIST);
          turnTo(90);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd - correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }

    stop();
    forwards();

    while (true) {
      correctionStart = System.currentTimeMillis();
      boolean detectedLeft = detectorL.lineDetected(), detectedRight = detectorR.lineDetected();
      if ((detectedLeft) || (detectedRight)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        stop();
        if (detectedRight) {
          isLeft = false;
        } else if (detectedLeft) {
          isLeft = true;
        }
        startPos = odometer.getXYT();
        // new Thread(odometer).start();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      deltaCorrection = correctionEnd - correctionStart;
      if (deltaCorrection < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    forwards();
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (detectorR.lineDetected()) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(-offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
          moveTo(5 + sensorOffset);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd - correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      while (true) {
        correctionStart = System.currentTimeMillis();
        if (detectorL.lineDetected()) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
          turnTo(offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
          moveTo(REVERSE_DIST + sensorOffset);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd - correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }

    odometer.setX(x * TILE_SIZE);
    odometer.setY(y * TILE_SIZE);
    odometer.setTheta(0);
  }


}
