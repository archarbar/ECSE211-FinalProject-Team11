package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Main class for methods that control the launcher
 * @author Victor
 *
 */
public class LauncherControl {

  /**
   * Launch the ping pong ball at the given speed in degrees/s.
   *
   * @param speed
   */
  public static void launch(int speed) {
    reset();
    try {
      Thread.sleep(1500);
      // Thread.sleep(5000);
    } catch (Exception e) {
      // do nothing
    }
    launcher1.setAcceleration(LAUNCHER_ACCELERATION);
    launcher1.setSpeed(speed);
    launcher2.setAcceleration(LAUNCHER_ACCELERATION);
    launcher2.setSpeed(speed);

    launcher1.rotate(LAUNCHER_ANGLE + extra_angle, true);
    launcher2.rotate(LAUNCHER_ANGLE + extra_angle, false);
    try {
      Thread.sleep(1500);
    } catch (Exception e) {
      // do nothing
    }
  }

  /**
   * Move the launcher to the ready position.
   */
  public static void reset() {
    launcher1.setSpeed(RESET_SPEED);
    launcher2.setSpeed(RESET_SPEED);

    launcher1.rotate(-LAUNCHER_ANGLE, true);
    launcher2.rotate(-LAUNCHER_ANGLE, false);
    try {
      Thread.sleep(500);
      // Thread.sleep(5000);
    } catch (Exception e) {
      // do nothing
    }
    launcher1.rotate(-extra_angle, true);
    launcher2.rotate(-extra_angle, false);

    launcher1.stop();
    launcher2.stop();
  }

  /**
   * lower the launcher arm without reloading.
   */
  public static void lowerArm() {
    launcher1.setSpeed(LOWER_SPEED);
    launcher2.flt();
//    launcher2.setSpeed(LOWER_SPEED);

    launcher1.rotate(-LOWER_ANGLE, true);
//    launcher2.rotate(-LOWER_ANGLE, false);

    launcher1.stop();
    launcher2.stop();
  }

  /**
   * raise the launcher arm without firing.
   */
  public static void raiseArm() {
    launcher1.setSpeed(LOWER_SPEED);
    launcher2.flt();
//    launcher2.setSpeed(LOWER_SPEED);

    launcher1.rotate(LOWER_ANGLE, true);
//    launcher2.rotate(LOWER_ANGLE, false);

    launcher1.flt();
//    launcher2.flt();
  }

  @Deprecated
  /**
   * Calculates the needed speed to launch the necessary distance.
   *
   * @return
   */
  public static int calculateSpeed() {
    int speed;
    double angleRad = Math.toRadians(LAUNCH_ANGLE);
    speed = (int) Math.sqrt((Math.pow((TARGET_DISTANCE * G * 100 / Math.cos(angleRad)), 2)
        / (2 * G * 100 * (LAUNCH_HEIGHT + TARGET_DISTANCE * Math.tan(angleRad)))));
    return speed;
  }

  /**
   * An improved method of calculating the speed of the launcher. Source:
   * https://en.wikipedia.org/wiki/Range_of_a_projectile.
   * 
   * @param distance
   * @return
   */
  public static int calculateSpeed(double distance) {
    int speed = 0;
    double angleRad = Math.toRadians(LAUNCH_ANGLE); // launch angle in radians
    double dSin = distance * Math.sin(2 * angleRad); // d sin(2*theta)
    double sin = Math.sin(angleRad); // sin(theta)
    double dividend = dSin * Math.sqrt(2 * G);
    double divisor = Math.sqrt(2 * dSin + LAUNCH_HEIGHT / (sin * sin));
    double quotient = dividend / divisor;
    speed = calculateMotorSpeed(quotient);
    return speed;
  }

  /*
   * Calculates required motor speed in degrees/sec given a tangential speed in cm/s.
   */
  public static int calculateMotorSpeed(double tangentialV) {
//    double angularV = tangentialV / LAUNCH_ARM_LENGTH;
    return (int) (tangentialV / LAUNCH_ARM_LENGTH);
  }
}
