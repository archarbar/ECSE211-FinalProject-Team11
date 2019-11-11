package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.Resources.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;

public class Main {
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  private static int[] launchTarget = new int[2];


  /**
   * The main method initializes all the threads and depending on user input, it runs either Falling edge or Rising edge
   * method.
   * 
   * @param args
   */
  public static void main(String[] args) {

    @SuppressWarnings("resource")
    SampleProvider usDistance = US_SENSOR.getMode("Distance");

    int buttonChoice;

    Odometer odometer = Odometer.getOdometer();
    Display odometryDisplay = new Display(lcd);

    do { // select method
      // clear the displays
      lcd.clear();

      lcd.drawString("Stationa-| Mobile  >", 0, 0);
      lcd.drawString("ry Launch| Launch   ", 0, 1);
      lcd.drawString("         |          ", 0, 2);
      lcd.drawString("    <    |     >    ", 0, 3);
      lcd.drawString("         |          ", 0, 4);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    // if mobile launch
    if (buttonChoice == Button.ID_RIGHT) {
      launchTarget = chooseTargetLocation();
      double[][] targetTile = new double[1][2];
      double[] destination = MainNavigation.waypointToLocation(launchTarget);
      targetTile[0][0] = destination[0];
      targetTile[0][1] = destination[1];

      // init localizer and navigation
      LightLocalizer lsLocalizer = new LightLocalizer(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD);
      MainNavigation navigation = new MainNavigation(leftMotor, rightMotor, TRACK, WHEEL_RAD, targetTile);
      UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, TRACK, WHEEL_RAD,
          UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
      initThreads(odometer, odometryDisplay);



      // Localization methods
      usLocalizer.mainMethod();
      lsLocalizer.mainMethod();

      // call navigation to go to destination
      navigation.run();
      Sound.beep();

    }
    // both options launch 5 times

    // launch
    LauncherControl.reset();
    for (int i = 0; i < 5; i++) {
      waitForPress();
      LauncherControl.launch(LAUNCH_SPEED);

    }
    System.exit(0);


  }

  /**
   * Enables the user to input a target location from the console.
   * 
   * @return
   */
  private static int[] chooseTargetLocation() {

    DecimalFormat numberFormat = new DecimalFormat("0");
    int index = 0;
    int pos[] = {0, 0, 0, 0};
    String spaces = " ";
    lcd.clear();
    lcd.drawString("(" + numberFormat.format(pos[0]) + numberFormat.format(pos[1]) + "," + numberFormat.format(pos[2])
        + numberFormat.format(pos[3]) + ")", 0, 0);
    lcd.drawString(spaces + "^", 0, 1);
    int buttonPress = Button.waitForAnyPress();
    while (buttonPress != Button.ID_ENTER) { // press enter to continue
      if (buttonPress == Button.ID_ESCAPE) {
        System.exit(0);
      } else if (buttonPress == Button.ID_LEFT) { // move left in the menu
        index--;
        if (index < 0) {
          index += 4;
        }
        index %= 4;

      } else if (buttonPress == Button.ID_RIGHT) { // move right in the menu
        index++;
        index %= 4;
      } else if (buttonPress == Button.ID_UP) { // increase coordinate
        pos[index]++;
        pos[index] %= 10;
      } else if (buttonPress == Button.ID_DOWN) { // decrease coordinate
        pos[index]--;
        if (pos[index] < 0) {
          pos[index] += 10;
        }
        pos[index] %= 10;
      }

      spaces = "";
      for (int i = 0; i <= index; i++) {
        spaces += " ";
      }
      if (index > 1) {
        spaces += " ";
      }
      lcd.clear();
      lcd.drawString("(" + numberFormat.format(pos[0]) + numberFormat.format(pos[1]) + "," + numberFormat.format(pos[2])
          + numberFormat.format(pos[3]) + ")", 0, 0);
      lcd.drawString(spaces + "^", 0, 1);



      buttonPress = Button.waitForAnyPress();
    }

    lcd.clear();
    int positions[] = new int[2];
    positions[0] = 10 * pos[0] + pos[1];
    positions[1] = 10 * pos[2] + pos[3];
    return positions;
  }

  /**
   * this method simply initiates the threads for the odometer and odometer display
   * 
   * @param odometer
   * @param odometryDisplay
   */
  private static void initThreads(Odometer odometer, Display odometryDisplay) {
    Thread odoThread = new Thread(odometer);
    odoThread.start();
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();
  }


  /**
   * Waits for any button to be pressed, if it's escape, exit the program early, otherwise wait 5 seconds to stabalize.
   */
  public static void waitForPress() {
    lcd.clear();
    Display.showText("Please press any ", "button to start  ", "the launcher    ", "                ",
        "                ");
    if (Button.waitForAnyPress() == Button.ID_ESCAPE) {
      System.exit(0);
    }
    lcd.clear();
    try {
      Thread.sleep(5000);
    } catch (Exception e) {

    }
  }

  /**
   * Returns array getLaunchTarget.
   * 
   * @return
   */
  public static int[] getLaunchTarget() {
    return launchTarget;
  }



}
