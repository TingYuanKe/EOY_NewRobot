package data;

public class InertialAcc {
	public double[] acc; //Input 3-axis values into double array
	
	public InertialAcc(double[] acc) {
		this.acc = acc;
	}
	
	public InertialAcc(double x, double y, double z) {
		// TODO Auto-generated constructor stub
		setAccRight_wrist(new double[] {x, y, z});
	}

	public double[] getAccRight_wrist() {
		return acc;
	}

	public void setAccRight_wrist(double[] acc) {
		this.acc = acc;
	}
}
