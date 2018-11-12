package data;

import java.util.ArrayList;

/**
* MTurn is turns which are classified as "Move" type.
* It extends the Turn object.
* It recodes acceleration of each axis and magnitude of 3-axis acceleration.
*
* @author  WeiChun
*/
public class MTurn extends Turn {
	
	private ArrayList<Double> acc = new ArrayList<Double>();
	private ArrayList<Double> acc_x = new ArrayList<Double>();
	private ArrayList<Double> acc_y = new ArrayList<Double>();
	private ArrayList<Double> acc_z = new ArrayList<Double>();
	
	public MTurn() {
		super();
	}
	
	public MTurn(double startTime, double endTime) {
		super(startTime, endTime);
	}

	public ArrayList<Double> getAcc() {
		return acc;
	}

	public void setAcc(ArrayList<Double> acc) {
		this.acc = acc;
	}
	
	public void addAcc(double acc) {
		this.acc.add(acc);
	}

	public ArrayList<Double> getAcc_x() {
		return acc_x;
	}

	public void setAcc_x(ArrayList<Double> acc_x) {
		this.acc_x = acc_x;
	}
	
	public void addAcc_x(double acc) {
		this.acc_x.add(acc);
	}

	public ArrayList<Double> getAcc_y() {
		return acc_y;
	}

	public void setAcc_y(ArrayList<Double> acc_y) {
		this.acc_y = acc_y;
	}
	
	public void addAcc_y(double acc) {
		this.acc_y.add(acc);
	}

	public ArrayList<Double> getAcc_z() {
		return acc_z;
	}

	public void setAcc_z(ArrayList<Double> acc_z) {
		this.acc_z = acc_z;
	}
	
	public void addAcc_z(double acc) {
		this.acc_z.add(acc);
	}
	
}
