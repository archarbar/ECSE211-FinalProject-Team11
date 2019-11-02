package ca.mcgill.ecse211.navigation;

public interface LineDetectorController {
  public enum Edge {NoEdge, RisingEdge, FallingEdge}
  
  public boolean lineDetected();
  
  public Edge edgeDetected();
  
  public double processCSData(double lightVal);
}
