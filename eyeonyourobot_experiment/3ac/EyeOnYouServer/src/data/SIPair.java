package data;

/**
* SIPair records similarity score of a combination of skeleton data and inertial data.
* Unique level of the skeleton is recorded as well. 
* Then it provides a customized sorting method, sorting with unique levels then similarity scores.
*
* @author  WeiChun
*/
public class SIPair implements Comparable<SIPair> {
	
	private int sid;
	private int iid;
	private double score;
	private double uniquev;

	public SIPair(int sid, int iid, double score, double value) {
		this.setSid(sid);
		this.setIid(iid);
		this.score = score;
		this.uniquev = value;
	}

	public int getSid() {
		return sid;
	}

	public void setSid(int sid) {
		this.sid = sid;
	}
	
	public int getIid() {
		return iid;
	}

	public void setIid(int iid) {
		this.iid = iid;
	}


	public double getScore() {
		return score;
	}

	public void setScore(double score) {
		this.score = score;
	}
	
	public double getUniquev() {
		return uniquev;
	}
	
	public void setUniquev(double uniquev) {
		this.uniquev = uniquev;
	}

	@Override
	public int compareTo(SIPair id) {
		if (this.uniquev > id.uniquev) {
			return -1;
		} else if (this.uniquev < id.uniquev) {
			return 1;
		} else {
			if (this.score > id.score) {
				return -1;
			} else if (this.score < id.score){
				return 1;
			} else {
				return 0;
			}
		}
	}

}
