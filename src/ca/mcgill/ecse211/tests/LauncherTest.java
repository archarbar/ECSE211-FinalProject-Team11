package ca.mcgill.ecse211.tests;

import ca.mcgill.ecse211.project.*;
import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.Resources.*;

public class LauncherTest {
  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    LauncherControl.reset();
    
    for(int i = 0; i<5; i++) {
      waitForPress();
//      LauncherControl.launch(LAUNCH_SPEED);
      LauncherControl.reset();
    }
    System.exit(0);
  }
  
  public static void waitForPress() {
    Display.showText("Please press any ", "button to start  ", "the launcher    ", "                ",
        "                ");
    if(Button.waitForAnyPress() == Button.ID_RIGHT) {
      System.exit(0);
    }
  }
}
