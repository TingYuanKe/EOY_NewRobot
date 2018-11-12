package data;

import java.util.ArrayList;

/**
* TurnList shows all "Move" and "Stop" turns extracted 
* from a time-series data in an interval.
*
* @author  WeiChun
*/
public class TurnList {
	
	private ArrayList<STurn> StillTurnList;
	private ArrayList<MTurn> MoveTurnList;
	
	public TurnList() {
		
	}
	
	public TurnList(ArrayList<STurn> stillTurnList, ArrayList<MTurn> moveTurnList) {
		this.StillTurnList = stillTurnList;
		this.MoveTurnList = moveTurnList;
	}
	
	public ArrayList<STurn> getStillTurnList() {
		return StillTurnList;
	}

	public void setStillTurnList(ArrayList<STurn> stillTurnList) {
		StillTurnList = stillTurnList;
	}

	public ArrayList<MTurn> getMoveTurnList() {
		return MoveTurnList;
	}

	public void setMoveTurnList(ArrayList<MTurn> moveTurnList) {
		MoveTurnList = moveTurnList;
	}
	
}
