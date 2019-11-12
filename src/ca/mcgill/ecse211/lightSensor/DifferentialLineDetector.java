package ca.mcgill.ecse211.lightSensor;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.robotics.SampleProvider;

public class DifferentialLineDetector implements LineDetectorController {



  // mean filter
  private static final int BUFFER_SIZE = 5;
  private double buffer[] = new double[BUFFER_SIZE];
  private int filterIndex = 0;
  private int filterCount = 0;

  // differential
  public final double THRESHOLD = 0.1;
  private double oldVal;

  // LineDetector
  private static final double MAX_DISTANCE = 3;
  private float[] sample;
  SampleProvider cs;
  Edge lastEdge = Edge.NoEdge;
  Double edgeX;
  Double edgeY;

  public DifferentialLineDetector(SampleProvider cs) {
    this.cs =cs; 
    lastEdge = Edge.NoEdge;
    sample = new float[cs.sampleSize()];
    cs.fetchSample(sample, 0);
    oldVal = sample[0];
  }

  @Override
  public boolean lineDetected() { // fallingEdge -> risingEdge
    Edge edge = edgeDetected();
    if (edge == Edge.NoEdge) {
      if (lastEdge == Edge.RisingEdge) {
        double[] position = Odometer.getOdometer().getXYT();
        double dx = position[0] - edgeX;
        double dy = position[1] - edgeY;
        if (Math.sqrt(dx * dx + dy * dy) > MAX_DISTANCE) {
          lastEdge = Edge.NoEdge;
        }
      }
      edgeX = null;
      edgeY = null;
      return false;
    } else if (edge == Edge.FallingEdge) { // detected dip in light
      double[] position = Odometer.getOdometer().getXYT();
      edgeX = position[0];
      edgeY = position[1];
      lastEdge = edge;
    } else if (edge == Edge.RisingEdge && lastEdge == Edge.FallingEdge) { // line detected
      // reset vars
      lastEdge = Edge.NoEdge;
      // leave edge X&Y to allow external access.
      return true;
    }
    return false;
  }

  @Override
  public double processCSData(double lightVal) {
    // optional meanFilter
    lightVal = meanFilter(lightVal);
    return lightVal - oldVal;
  }

  protected double meanFilter(double lightVal) {
    buffer[filterIndex++] = lightVal;
    filterIndex %= buffer.length;
    if (filterCount < buffer.length) {
      filterCount++;
    }
    float mean = 0;
    for (int i = 0; i < buffer.length && i < filterCount; i++) {
      mean += buffer[i];
    }
    mean /= filterCount;
    return mean;
  }

  @Override
  public Edge edgeDetected() {
    Edge edge = Edge.NoEdge;
    cs.fetchSample(sample, 0);
    double newVal = sample[0];

    double difference = processCSData(newVal);
    if (difference < THRESHOLD) {
      edge = Edge.FallingEdge;
    } else if (difference > THRESHOLD) {
      edge = Edge.RisingEdge;
    }

    oldVal = newVal; // update the previous value.
    return edge;
  }

  /**
   * @return the oldVal
   */
  public double getOldVal() {
    return oldVal;
  }

  /**
   * @return the edgeX
   */
  public double getEdgeX() {
    return edgeX;
  }

  /**
   * @return the edgeY
   */
  public double getEdgeY() {
    return edgeY;
  }


}
