package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.lightSensor.*;

/**
 * localises the robot using colour sensors to detect lines arranged in a grid in order to localise angle and position.
 */
public class ModifiedLightLocalizer extends PlainNavigation {
  private static final long CORRECTION_PERIOD = 5;
  private static final int REVERSE_DIST = 3;
  private double distance;
  private double[] startPos;
  private Odometer odometer;
  private double offTheta;
  private boolean isLeft = false;
  private LineDetectorController detectorL; // use a red light to compare luminence level
  private LineDetectorController detectorR;

  /**
   * initialises 2 light sensors and gets the odometer.
   */
  public ModifiedLightLocalizer(LineDetectorController detectorL, LineDetectorController detectorR) {
    this.odometer = Odometer.getOdometer();
    this.detectorL = detectorL;
    this.detectorR = detectorR;
  }

  /**
   * tells the robot to localise and set the given point as its given location.
   * 
   * @param x
   * @param y
   */
  public void localize(int x, int y) {
    findDistance(x, y);
  }

  /**
   * Localizes the robot then sets its final position to the given values.
   * 
   * @param x
   * @param y
   */
  private void findDistance(int x, int y) {
    odometer.setXYT(0, 0, 0);
    long correctionStart, correctionEnd;
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
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
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
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          turnTo(-offTheta);
          moveTo(-REVERSE_DIST);
          turnTo(90);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
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
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          turnTo(offTheta);
          moveTo(-REVERSE_DIST);
          turnTo(90);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
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
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
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
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          turnTo(-offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
          moveTo(5 + sensorOffset);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
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
          LCD.drawString("theta:  " + String.valueOf(offTheta), 0, 4);
          LCD.drawString("distance:  " + String.valueOf(distance), 0, 5);
          turnTo(offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
          moveTo(REVERSE_DIST + sensorOffset);
          break;
        }
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }

    // while (true) {
    // correctionStart = System.currentTimeMillis();
    // colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
    // colorSampleProviderR.fetchSample(csDataR, 0);
    // if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) {
    // // if light read by sensor is smaller (darker) than red light, eg., black lines
    // // Sound.beep();
    // moveTo(sensorOffset);
    // leftMotor.rotate(-convertAngle(90.0), true);
    // rightMotor.rotate(convertAngle(90.0), false);
    // moveTo(5+sensorOffset);
    // break;
    // // motor.stop();
    // }
    // correctionEnd = System.currentTimeMillis();
    // if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
    // try {
    // Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
    // } catch (InterruptedException e) {
    // e.printStackTrace();
    // }
    // }
    // }

    odometer.setX(x * TILE_SIZE);
    odometer.setY(y * TILE_SIZE);
    odometer.setTheta(0);
  }


}
