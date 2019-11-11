package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.Lab3.Resources.odometer;
import static ca.mcgill.ecse211.lab5.Resources.TRACK;
import static ca.mcgill.ecse211.lab5.Resources.US_SENSOR;
import static ca.mcgill.ecse211.lab5.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.lab5.Resources.leftMotor;
import static ca.mcgill.ecse211.lab5.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.*;
import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.Display;
import ca.mcgill.ecse211.project.LightLocalizer;
import ca.mcgill.ecse211.project.UltrasonicLocalizer;
import ca.mcgill.ecse211.navigation.LineNavigation;
import ca.mcgill.ecse211.navigation.Navigation;
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
    betaDemo();
  }

  private static void localize() {
    SampleProvider usDistance = US_SENSOR.getMode("Distance");
    LightLocalizer lsLocalizer = new LightLocalizer();
    UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(UltrasonicLocalizer.edgeType.FallingEdge, usDistance);
    usLocalizer.mainMethod();
    lsLocalizer.mainMethod();
    
  }

  private static void startOdometer() {
    odometer = Odometer.getOdometer();
    new Thread(odometer).start();
  }
  
  private static void navigateThroughTunnel() {
    Point tunnelEntr = Navigation.findTunnelEntrance(TNG_LL, TNG_UR);
    LineNavigation.travelTo(tunnelEntr.x, tunnelEntr.y);
   
    //find angle of tunnel
    double aveX, aveY, theta;
    aveX = (TNG_LL.x+TNG_UR.x)/2;
    aveY = (TNG_LL.y+TNG_UR.y)/2;
    theta = Navigation.angleToTarget(aveX, aveY);
    Navigation.turnTo(theta);
    LauncherControl.lowerArm();
    Navigation.moveTo(3*TILE_SIZE);
    LauncherControl.raiseArm();
  }
  
  private static void navigateToLaunch() {
    LineNavigation.travelTo(BIN.x, BIN.y);
    Navigation.turnToHeading(TNR_UR_x);
  }
  
  private static void importData() {}
  
  private static void launch(int numLaunches, int speed) {
    for(int i=0;i<numLaunches;++i) {
      LauncherControl.launch(speed);
    }
  }
  
  private static void mainFlow() {

  }

  private static void betaDemo() {
    //import wifi data
    importData();
    //odometry start
    startOdometer();
    
    //localize
    localize();
    Sound.beep();
    
    //navigate to tunnel
    navigateThroughTunnel();
    
    //navigate to specified coords
    //turn to specified angle
    navigateToLaunch();
    Sound.beep();
    Sound.beep();
    Sound.beep();
    //launch
    int maxSpeed = 1300;
    launch(1, maxSpeed);
    Sound.beep();
    
    System.exit(0);

  }

}
