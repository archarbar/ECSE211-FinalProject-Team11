package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.robotics.SampleProvider;

/**
 * localises the robot using colour sensors to detect lines arranged in a grid in order to localise angle and position.
 */
public class LightLocalizer extends PlainNavigation {

  /**
   * correction period in milliseconds
   */
  private static final long CORRECTION_PERIOD = 5;

  /**
   * back off distance for robot in cm
   */
  private static final int REVERSE_DIST = 3;

  /**
   * left light sensor data
   */
  private float[] csDataL;

  /**
   * right light sensor data
   */
  private float[] csDataR;

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
   * left light sensor sample provider
   */
  private SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level

  /**
   * right light sensor sample provider
   */
  private SampleProvider colorSampleProviderR = colorSensorR.getRedMode();

  /**
   * red light intensity to compare read colors
   */
  private static final float LINE_RED_INTENSITY = 0.3f; // cast to float since default is double

  /**
   * initialises 2 light sensors and gets the odometer.
   */
  public LightLocalizer() {
    this.odometer = Odometer.getOdometer();
    csDataL = new float[colorSensorL.sampleSize()];
    csDataR = new float[colorSensorR.sampleSize()];
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
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        // sound alert to know when the sensor hits a line
        stop();
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
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
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
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
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
          stop();
          distance = (odometer.getXYT())[1] - startPos[1];
          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
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
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        // sound alert to know when the sensor hits a line
        stop();
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
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
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
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
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
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
