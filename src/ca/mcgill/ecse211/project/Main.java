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
import ca.mcgill.ecse211.navigation.GridRectangle;
import ca.mcgill.ecse211.navigation.LineNavigation;
import ca.mcgill.ecse211.navigation.Navigation;
import ca.mcgill.ecse211.navigation.PlainNavigation;
import ca.mcgill.ecse211.odometer.Odometer;

public class Main {
  private static Odometer odometer;
  private static Point TNG_LL, TNG_UR, BIN;
  private static int TNR_UR_x;


  /**
   * The main method initializes all the threads and depending on user input, it runs either Falling edge or Rising edge
   * method.
   * 
   * @param args
   */
  public static void main(String[] args) {
    // betaDemo();
    // startOdometer();
    // localize();
    // localizationTimeTest();
    // lineNavigationTest();
    // plainNavigationTest();
    // tunnelTest();
    launchTest();

  }

  private static void localize(int x, int y) {
    SampleProvider usDistance = US_SENSOR.getMode("Distance");
    LightLocalizer lsLocalizer = new LightLocalizer();
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
    usLocalizer.mainMethod();
    lsLocalizer.mainMethod(x, y);

  }

  private static void startOdometer() {
    odometer = Odometer.getOdometer();
    new Thread(odometer).start();
  }

  private static void navigateThroughTunnel(Navigation navigator) {
    Point tunnelEntr = Navigation.findTunnelEntrance(TNG_LL, TNG_UR);
    navigator.travelTo(tunnelEntr.x, tunnelEntr.y);

    // find angle of tunnel
    double aveX, aveY, theta;
    aveX = (TNG_LL.x + TNG_UR.x) / 2;
    aveY = (TNG_LL.y + TNG_UR.y) / 2;
    theta = Navigation.angleToTarget(aveX, aveY);
    Navigation.turnTo(theta);
    LauncherControl.lowerArm();
    Navigation.moveTo(3 * TILE_SIZE);
    LauncherControl.raiseArm();
  }

  private static void navigateToLaunch(Navigation navigator) {
    navigator.travelTo(BIN.x, BIN.y);
    Navigation.turnToHeading(TNR_UR_x);
  }

  private static void launch(int numLaunches, int speed) {
    for (int i = 0; i < numLaunches; ++i) {
      LauncherControl.launch(speed);
    }
  }

  private static void mainFlow() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;

  }

  private static void betaDemo() {
    // set to silent verification
    Resources.SILENT_VERIFICATION = true;
    // import wifi data is done by default
    // odometry start
    startOdometer();

    // localize
    localize(1, 1);
    Sound.beep();

    // navigate to tunnel
    navigateThroughTunnel(new LineNavigation());

    // navigate to specified coords
    // turn to specified angle
    navigateToLaunch(new LineNavigation());
    Sound.beep();
    Sound.beep();
    Sound.beep();
    // launch
    int maxSpeed = 1300;
    launch(1, maxSpeed);
    Sound.beep();

    System.exit(0);

  }

  private static void lineNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    Navigation navigator = new LineNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }

  private static void plainNavigationTest() {
    double x = 3 * TILE_SIZE, y = 4 * TILE_SIZE;
    Navigation navigator = new PlainNavigation();
    navigator.safeArea = new GridRectangle(0, 0, 15, 15);
    navigator.travelTo(x, y);
    System.exit(0);
  }

  private static void localizationTimeTest() {
    long startTime = System.currentTimeMillis();
    localize(1, 1);
    long endTime = System.currentTimeMillis();
    long totalTime = endTime - startTime;
    if (totalTime > 30 * 1000)
      Sound.beepSequence();
    else
      Sound.beepSequenceUp();
  }

  private static void tunnelTest() {
    TNG_LL = new Point(2, 3);
    TNG_UR = new Point(4, 4);
    navigateThroughTunnel(new PlainNavigation());
  }

  private static void launchTest() {
    int maxSpeed = 1300;
    launch(3, maxSpeed);
  }
}
