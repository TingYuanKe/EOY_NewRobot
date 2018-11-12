package data;

/**
* Turn is the basic unit in EOY.
* It records starting time and ending time in 0.1-second scale of each turn. 
*
* @author  WeiChun
*/
public class Turn {
	
	private double startTime;
	private double endTime;
	
	public Turn() {
		
	}
	
	public Turn(double startTime, double endTime) {
		this.startTime = startTime;
		this.endTime = endTime;
	}

	public double getStartTime() {
		return startTime;
	}

	public void setStartTime(int startTime) {
		this.startTime = startTime;
	}

	public double getEndTime() {
		return endTime;
	}

	public void setEndTime(int endTime) {
		this.endTime = endTime;
	}
	
}
