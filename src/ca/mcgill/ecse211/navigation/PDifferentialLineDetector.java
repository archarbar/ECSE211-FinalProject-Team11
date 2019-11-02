package ca.mcgill.ecse211.navigation;

import lejos.hardware.sensor.EV3ColorSensor;

public class PDifferentialLineDetector extends DifferentialLineDetector {

  public final double THRESHOLD = 0.30; //The difference has to be greater than 30%
  
  public PDifferentialLineDetector(EV3ColorSensor cs) {
    super(cs);
  }

  @Override
  public double processCSData(double newVal) {
    double oldVal = getOldVal();
    
    //optionally use mean filter.
//    newVal = meanFilter();
    return (newVal - oldVal)/newVal;
  }
}
