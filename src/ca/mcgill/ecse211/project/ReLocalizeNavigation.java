package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.robotics.SampleProvider;

/**
 * A navigator that actively corrects its heading and position.
 *
 *
 * @author Matthew
 *
 */
public class ReLocalizeNavigation extends WaggleNavigation {

  @Override
  public void travelTo(double x, double y) {
    // colorSensorR.close();
    // colorSensorL.close();
    do {
    new PlainNavigation().travelTo(x, y);
    } while(avoided);
  }

  public void reLocalize(double x, double y) {
    long correctionStart, correctionEnd, deltaCorrection;
    csDataR = new float[colorSensorR.sampleSize()];
    csDataL = new float[colorSensorL.sampleSize()];
    SampleProvider colorSampleProviderL = colorSensorL.getRedMode(); // use a red light to compare luminence level
    SampleProvider colorSampleProviderR = colorSensorR.getRedMode();
    
    turnTo(Navigation.angleToTarget(x, y));
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
    
    moveTo(-TILE_SIZE/2+sensorOffset);
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
    moveTo(-TILE_SIZE/2+sensorOffset);
    turnTo(Navigation.angleToTarget(x,y));
  }
//    double[] xyt = Odometer.getOdometer().getXYT();
//    double dx, dy;
//    int direction, axis;
//    dx = Math.abs(xyt[0]-x);
//    dy = Math.abs(xyt[1]-y);
//    if(dx>dy) {
//      direction = 0;
//      if(xyt[0]>x) {
//        axis = 1;
//      } else {
//        axis = -1;
//      }
//      waggle(axis, direction);
//      moveTo(-TILE_SIZE/2);
//
//      direction = 1;
//      axis = 1;
//      turnToHeading(0);
//      waggle(axis, direction);
//      moveTo(-TILE_SIZE/2);
//      turnTo(Navigation.angleToTarget(x, y));
//
//    } else {  //dy>dx
//      direction = 1;
//      if(xyt[1]>y) {
//        axis = 1;
//      } else {
//        axis = -1;
//      }
//      waggle(axis, direction);
//      moveTo(-TILE_SIZE/2);
//
//      direction = 0;
//      axis = 1;
//      turnToHeading(90);
//      waggle(axis, direction);
//      moveTo(-TILE_SIZE/2);
//      turnTo(Navigation.angleToTarget(x, y));
//    }
//  }
  private void left() {
    leftMotor.setSpeed(ROTATE_SPEED);
    leftMotor.forward();
  }
  private void right() {
    rightMotor.setSpeed(ROTATE_SPEED);
    rightMotor.forward();
  }
}
