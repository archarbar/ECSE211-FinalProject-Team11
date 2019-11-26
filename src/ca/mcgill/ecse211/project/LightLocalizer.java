package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * localises the robot using colour sensors to detect lines arranged in a grid in order to localise angle and position.
 */
public class LightLocalizer extends PlainNavigation {

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
   * red light intensity to compare read colors
   */
  private static final float LINE_RED_INTENSITY = 0.3f; // cast to float since default is double

  /**
   * gets the odometer.
   */
  public LightLocalizer() {
    this.odometer = Odometer.getOdometer();
  }

  /**
   * tells the robot to localise and set the given point as its given location.
   * 
   * @param x
   * @param y
   */
  public void localize(int x, int y, double theta) {
    findDistance(x, y, theta);
  }

  /**
   * Localizes the robot then sets its final position to the given values.
   * 
   * @param x
   * @param y
   */
  private void findDistance(int x, int y, double theta) {
    /**
     * left light sensor sample provider
     */
    SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level

    /**
     * right light sensor sample provider
     */
    SampleProvider colorSampleProviderR = colorSensorR.getRedMode();
    
    /**
     * initialize the two light sensors
     */
    csDataL = new float[colorSensorL.sampleSize()];
    csDataR = new float[colorSensorR.sampleSize()];
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
      colorSampleProviderL.fetchSample(csDataL, 0); // get data from sensor
      colorSampleProviderR.fetchSample(csDataR, 0);
      if ((csDataL[0] < LINE_RED_INTENSITY) || (csDataR[0] < LINE_RED_INTENSITY)) { // if light read by sensor is
                                                                                    // smaller (darker) than red light,
                                                                                    // eg., black lines
        // sound alert to know when the sensor hits a line
//        Sound.beep();
        
        stop();
        if (csDataR[0] < LINE_RED_INTENSITY) {
          isLeft = false;
        } else if (csDataL[0] < LINE_RED_INTENSITY) {
          isLeft = true;
        }
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        startPos = odometer.getXYT();
//        System.out.println("y:"+startPos[1]);
        
        // new Thread(odometer).start();
        break;
      }
      correctionEnd = System.currentTimeMillis();
      deltaCorrection = correctionEnd-correctionStart;
      if (deltaCorrection < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
    
    if (isLeft) {
      while (true) {
        correctionStart = System.currentTimeMillis();
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          stop();
//          Sound.beep();
//          System.out.println("y:"+Odometer.getOdometer().getXYT()[1]);
//          distance = (odometer.getXYT())[1] - startPos[1];
//          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
//          System.out.println("dis:"+distance);
//          System.out.println("theta:"+offTheta);
//          turnTo(-offTheta);
//          Sound.beepSequenceUp();
//          moveTo(-REVERSE_DIST);
//          Sound.beepSequence();
          break;
        }
//        forwards();
        right();
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd-correctionStart;
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
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
          stop();
//          Sound.beep();
//          System.out.println("y:"+Odometer.getOdometer().getXYT()[1]);
//          distance = (odometer.getXYT())[1] - startPos[1];
//          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
//          System.out.println("dis:"+distance);
//          System.out.println("theta:"+offTheta);
//          turnTo(offTheta);
//          Sound.beepSequenceUp();
//          moveTo(-REVERSE_DIST);
//          Sound.beepSequence();
          break;
        }
//        forwards();
        left();
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd-correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    }
    moveTo(sensorOffset);
    turnTo(90);
    stop();
    forwards();
    while (true) {
      correctionStart = System.currentTimeMillis();
      int loc = 0;

      colorSampleProviderL.fetchSample(csDataL, loc); // get data from sensor
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
        try {
          Thread.sleep(10);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        startPos = odometer.getXYT();
        break;
      }
      
      correctionEnd = System.currentTimeMillis();
      deltaCorrection = correctionEnd-correctionStart;
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
        colorSampleProviderR.fetchSample(csDataR, 0);
        if (csDataR[0] < LINE_RED_INTENSITY) {
          stop();
//          distance = (odometer.getXYT())[1] - startPos[1];
//          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
//          turnTo(-offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
//          moveTo(sensorOffset);//+reverse_dist
          break;
        }
//        forwards();
        right();
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd-correctionStart;
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
        colorSampleProviderL.fetchSample(csDataL, 0);
        if (csDataL[0] < LINE_RED_INTENSITY) {
          stop();
//          distance = (odometer.getXYT())[1] - startPos[1];
//          offTheta = Math.toDegrees(Math.atan(distance / LSwidth));
//          turnTo(offTheta);
          moveTo(sensorOffset);
          turnTo(-90);
//          moveTo(sensorOffset); //+reverse_dist
          break;
        }
//        forwards();
        left();
        correctionEnd = System.currentTimeMillis();
        deltaCorrection = correctionEnd-correctionStart;
        if (deltaCorrection < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - deltaCorrection);
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
    odometer.setTheta(theta);
    
  }
}
