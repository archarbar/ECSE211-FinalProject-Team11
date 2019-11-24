package ca.mcgill.ecse211.project;

import lejos.hardware.sensor.EV3ColorSensor;

/**
 * A line detector that detects lines using a difference in the light levels proportional to their average values.
 * 
 * @author Matthew
 *
 */
public class PDifferentialLineDetector extends DifferentialLineDetector {
  
  /**
   * The difference has to be greater than 30%
   */
  public final double THRESHOLD = 0.30;

  /**
   * Calls the DifferentialLineDetector constructor.
   * 
   * @param colorsensor the colour sensor you want the line detector to track.
   */
  public PDifferentialLineDetector(EV3ColorSensor cs) {
    super(cs);
  }

  /**
   * Processes the data proportional to the difference in values.
   * 
   * @param newColourVal the most recent light value detected.
   * @return light value
   */
  @Override
  public double processCSData(double newVal) {
    double oldVal = getOldVal();

    // optionally use mean filter.
    // newVal = meanFilter();
    return (newVal - oldVal) / (newVal + oldVal) * 2;
  }
}
