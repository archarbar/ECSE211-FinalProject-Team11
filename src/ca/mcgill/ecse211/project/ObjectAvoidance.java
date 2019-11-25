package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
/**
 * The class to use to avoid objects.
 * 
 * @author Matthew
 *
 */
public class ObjectAvoidance implements Runnable{
  private boolean running = false;
  
  Navigation navigator;
  
  public ObjectAvoidance(Navigation navigator) {
    this.navigator = navigator;
  }
  
  public void stop() {
    running = false;
  }

  @Override
  public void run() {
    running = true;
    while (running) {
      long startTime = System.currentTimeMillis();
      boolean detected = false;
      //TODO: detect object
      if (detected) {
        navigator.avoided = true;
        navigator.avoidance();
        Navigation.stop();
        avoid();
        navigator.avoidanceover();
      }
      
      long endTime = System.currentTimeMillis();
      if (endTime-startTime < AVOIDANCE_PERIOD) {
        try {
          Thread.sleep(AVOIDANCE_PERIOD - (endTime - startTime));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }
  
  private static void avoid() {
    boolean avoiding = true;
    while(avoiding) {
      double[] xyt = Odometer.getOdometer().getXYT();
      //TODO: determine direction to go (-1 for left, 1 for right)
      int direction = 0;
      
      //TODO turn until rising edge.
      
      double[] xyt2 = Odometer.getOdometer().getXYT();
      double diffT = Math.abs(xyt[2]-xyt2[2]);
      if (diffT<90) {
        Navigation.turnTo(direction*(90-diffT));
        
        //distance to travel sin(Math.toRadians(difference))*distance detected = 
//        move(distance to travel);
//        turn back 90
      }
    }
  }
}
