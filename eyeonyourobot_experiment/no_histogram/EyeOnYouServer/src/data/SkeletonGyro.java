package data;

public class SkeletonGyro {
	public double[] gyro; //Input 3-axis values into double array
	
	public SkeletonGyro(double[] gyro) {
		this.gyro = gyro;
	}
	
	public SkeletonGyro(double x, double y, double z) {
		setGyroRight_wrist(new double[] {x, y, z});
	}

	public double[] getGyroRight_wrist() {
		return gyro;
	}

	public void setGyroRight_wrist(double[] gyro) {
		this.gyro = gyro;
	}
}
