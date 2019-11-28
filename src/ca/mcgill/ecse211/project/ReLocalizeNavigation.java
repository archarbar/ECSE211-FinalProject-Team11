package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.robotics.SampleProvider;

/**
 * A navigator that can corrects its heading and position.
 * It will first travel directly to its destination, then correct its position and 
 * angle within the destination tile.
 *
 *
 * @author Matthew
 *
 */
public class ReLocalizeNavigation extends WaggleNavigation {

  public double distance;
  @Override
  public void travelTo(double x, double y) {
    // colorSensorR.close();
    // colorSensorL.close();
    distance = calculateDistanceTo(x,y);
    do {
      new PlainNavigation().travelTo(x, y);
    } while(avoided);
  }

  /**
   * relocalizes the robot to be called after travelling.
   * @param x value of the point to face at the end.
   * @param y value of the point to face at the end.
   */
  public void reLocalize(double x, double y) {
    long correctionStart, correctionEnd, deltaCorrection;
    csDataR = new float[colorSensorR.sampleSize()];
    csDataL = new float[colorSensorL.sampleSize()];
    SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level
    SampleProvider colorSampleProviderR = colorSensorR.getRedMode();
    
    turnTo(Navigation.angleToTarget(x, y));
    moveTo(-5);
    try {
      Thread.sleep(10);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    turnTo(180);

    
    
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
    Odometer odometer = Odometer.getOdometer();
    double[] xyt = odometer.getXYT();
    xyt[2] = Math.round(xyt[2]/90)*90;
    if (xyt[2] == 90 || xyt[2] == 270) {
      xyt[0] = Math.round(xyt[0]/TILE_SIZE)*TILE_SIZE;
    } else {
      xyt[1] = Math.round(xyt[1]/TILE_SIZE)*TILE_SIZE;
    }
    odometer.setXYT(xyt[0], xyt[1], xyt[2]);
    
    moveTo(-TILE_SIZE*0.3+sensorOffset);
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
    xyt = odometer.getXYT();
    xyt[2] = Math.round(xyt[2]/90)*90;
    if (xyt[2] == 90 || xyt[2] == 270) {
      xyt[0] = Math.round(xyt[0]/TILE_SIZE)*TILE_SIZE;
    } else {
      xyt[1] = Math.round(xyt[1]/TILE_SIZE)*TILE_SIZE;
    }
    odometer.setXYT(xyt[0], xyt[1], xyt[2]);
    moveTo(-TILE_SIZE/2+sensorOffset-1.5);
    
    turnTo(90);
    moveTo(TILE_SIZE*0.2);
  }  
}
