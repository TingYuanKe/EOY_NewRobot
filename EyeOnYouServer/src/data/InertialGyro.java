package data;

public class InertialGyro {
	public double[] gyro; //Input 3-axis values into double array
	
	public InertialGyro(double[] gyro) {
		this.gyro = gyro;
	}
	
	public InertialGyro(double x, double y, double z) {
		setGyroRight_wrist(new double[] {x, y, z});
	}

	public double[] getGyroRight_wrist() {
		return gyro;
	}

	public void setGyroRight_wrist(double[] gyro) {
		this.gyro = gyro;
	}
}
