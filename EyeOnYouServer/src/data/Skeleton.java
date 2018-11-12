package data;

/**
* Skeleton object is used to record joints positions in 3-axis in every sample.
*
* @author  WeiChun
*/
public class Skeleton {
	
	private double[] right_wrist;
//	private double[] acc; //Input 3-axis values into double array
	
	public Skeleton() {
		
	}
	
	public Skeleton(double x1, double y1, double z1, double x2, double y2, double z2) {
		setRight_wrist(new double[] {x1, y1, z1, x2, y2, z2});
	}

	public double[] getRight_wrist() {
		return right_wrist;
	}

	public void setRight_wrist(double[] right_wrist) {
		this.right_wrist = right_wrist;
	}
	
}
