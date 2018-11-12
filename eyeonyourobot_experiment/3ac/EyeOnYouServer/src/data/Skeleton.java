package data;

/**
* Skeleton object is used to record joints positions in 3-axis in every sample.
*
* @author  WeiChun
*/
public class Skeleton {
	
	private double[] right_wrist;
	
	public Skeleton() {
		
	}
	
	public Skeleton(double x, double y, double z) {
		setRight_wrist(new double[] {x, y, z});
	}

	public double[] getRight_wrist() {
		return right_wrist;
	}

	public void setRight_wrist(double[] right_wrist) {
		this.right_wrist = right_wrist;
	}
	
}
