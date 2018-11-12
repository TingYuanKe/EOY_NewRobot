package preprocess;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import data.Skeleton;

/**
* This class is used to process skeleton data into some spatial information.
*
* @author  WeiChun
*/
public class SkeletonInfo {
	
	/**
	* process joint positions into distance
	*   
	* @param: list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* @param: frame data number per segment(0.2sec) , 20*0.2 = 4 as default
	* 
	* @return: arrays of displacements in each axis
	*/
	public static double[][] getDistanceList(ArrayList<Skeleton> jointspos, int sampleingRateOfSkn, int frameSizePerSeg) {
		//(t) Position frames per segment are calculated into (t-1) Distance data per segment ( t=4 as default)
		double [][] distance = new double[3][jointspos.size()/frameSizePerSeg * (frameSizePerSeg-1)];
		int len = 0;
		
		for (int i = 0; i < jointspos.size(); i+=frameSizePerSeg) {
			if (i+frameSizePerSeg > jointspos.size()) {
				break;
			}
			List<Skeleton>subjointspos = jointspos.subList(i, i+frameSizePerSeg);
			for (int j = 0; j < (frameSizePerSeg-1); j++) {
				distance[0][len] = subjointspos.get(j+1).getRight_wrist()[0] - subjointspos.get(j).getRight_wrist()[0];
				distance[1][len] = subjointspos.get(j+1).getRight_wrist()[1] - subjointspos.get(j).getRight_wrist()[1];
				distance[2][len] = subjointspos.get(j+1).getRight_wrist()[2] - subjointspos.get(j).getRight_wrist()[2];
				len++;
			}
		}
		
		return distance;
	}
	
	/**
	* process joint positions into velocity
	*   
	* @param: list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* @param: frame data number per segment(0.2sec) , 20*0.2 = 4 as default
	* 
	* @return: arrays of velocity in each axis
	*/
	public static double[][] getSpeedList(ArrayList<Skeleton> jointspos, int sampleingRateOfSkn, int frameSizePerSeg) {
		//(t-1) Distance data per segment are calculated into (t-1) Velocity data per segment ( t=4 as default)
		double [][] distance = getDistanceList(jointspos, sampleingRateOfSkn,frameSizePerSeg);
		double [][] speed = new double[3][distance[0].length];
		
		for (int i = 0; i < distance[0].length; i++) {
			for (int axis = 0; axis < 3; axis++) {
				speed[axis][i] = distance[axis][i] * sampleingRateOfSkn;
			}
		}
		
		return speed;
	}
	
	/**
	* process joint positions into acceleration
	*   
	* @param: list of Skeleton objects
	* 
	* @return: arrays of acceleration in each axis
	*/
	public static double[][] getAccList(ArrayList<Skeleton> certainJointsPos, int sampleingRateOfSkn ,int frameSizePerSeg) {
		//(t-1) Velocity data per segment are calculated into (t-2) Acceleration data per segment ( t=4 as default)
		double[][] speed = getSpeedList(certainJointsPos, sampleingRateOfSkn, frameSizePerSeg);
		double[][] acc = new double[3][speed[0].length/(frameSizePerSeg-1)*(frameSizePerSeg-2)];
		int len = 0;
		
		for (int i = 0; i < speed[0].length; i += (frameSizePerSeg-1)) {
			if(i+(frameSizePerSeg-1) > speed[0].length) {
				break;
			}
			double[] subspeed_x = Arrays.copyOfRange(speed[0], i, i + (frameSizePerSeg-1));
			double[] subspeed_y = Arrays.copyOfRange(speed[1], i, i + (frameSizePerSeg-1));
			double[] subspeed_z = Arrays.copyOfRange(speed[2], i, i + (frameSizePerSeg-1));
			for (int j = 0; j < (frameSizePerSeg-2); j++) {
				acc[0][len] = (subspeed_x[j + 1] - subspeed_x[j]) * sampleingRateOfSkn;
				acc[1][len] = (subspeed_y[j + 1] - subspeed_y[j]) * sampleingRateOfSkn;
				acc[2][len] = (subspeed_z[j + 1] - subspeed_z[j]) * sampleingRateOfSkn;
				len++;
			}
		}
		
		return acc;
		
	}
	
}
