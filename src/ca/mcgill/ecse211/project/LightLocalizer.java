package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
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
   * our odometer
   */
  private Odometer odometer;

  /**
   * boolean to know which side robot is facing
   */
  private boolean isLeft = false;

  /**
   * red light intensity to compare read colors
   */
  private static final float LINE_RED_INTENSITY = 0.35f; // cast to float since default is double

  /**
   * gets the odometer.
   */
  public LightLocalizer() {
    this.odometer = Odometer.getOdometer();
  }

  /**
   * tells the robot to localise and set the given point as its given location.
   * 
   * @param x position to localize to.
   * @param y position to localize to.
   * @param theta angle to set at end of localization.
   */
  public void localize(int x, int y, double theta) {
    findDistance(x, y, theta);
  }

  /**
   * Localizes the robot then sets its final position to the given values.
   * 
   * @param x position to localize to.
   * @param y position to localize to.
   * @param theta angle to set at end of localization.
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
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
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
//        System.out.println();
        if (csDataR[0] < LINE_RED_INTENSITY) {
          stop();
          break;
        }
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
          break;
        }
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
    try {
      Thread.sleep(50);
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }
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
          Thread.sleep(CORRECTION_PERIOD);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
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
          moveTo(sensorOffset);
          turnTo(-90);
          break;
        }
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
          moveTo(sensorOffset);
          turnTo(-90);
          break;
        }
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

    odometer.setX(x * TILE_SIZE);
    odometer.setY(y * TILE_SIZE);
    odometer.setTheta(theta);
    
  }
}
