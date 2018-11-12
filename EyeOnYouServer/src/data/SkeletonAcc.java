package data;

public class SkeletonAcc {
	public double[] acc; //Input 3-axis values into double array
	
	public SkeletonAcc(double[] acc) {
		this.acc = acc;
	}
	
	public SkeletonAcc(double x, double y, double z) {
		setAccRight_wrist(new double[] {x, y, z});
	}

	public double[] getAccRight_wrist() {
		return acc;
	}

	public void setAccRight_wrist(double[] acc) {
		this.acc = acc;
	}
}
