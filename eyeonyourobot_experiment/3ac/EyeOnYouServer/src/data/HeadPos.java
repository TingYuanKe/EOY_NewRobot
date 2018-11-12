package data;

/**
* HeadPos is used to record head's 2D coordinate in each video frame.
*
* @author  WeiChun
*/
public class HeadPos {
	
	private double x;
	private double y;
	
	public HeadPos() {
	
	}
	
	public HeadPos(double x, double y) {
		this.setX(x);
		this.setY(y);
	}
	
	public double getX() {
		return x;
	}
	
	public void setX(double x) {
		this.x = x;
	}
	
	public double getY() {
		return y;
	}
	
	public void setY(double y) {
		this.y = y;
	}
	
}
