package ca.mcgill.ecse211.lightSensor;

public interface LineDetectorController {
  public enum Edge {
    NoEdge, RisingEdge, FallingEdge
  }

  public boolean lineDetected();

  public Edge edgeDetected();

  public double processCSData(double lightVal);

  public double getEdgeX();

  public double getEdgeY();

}
