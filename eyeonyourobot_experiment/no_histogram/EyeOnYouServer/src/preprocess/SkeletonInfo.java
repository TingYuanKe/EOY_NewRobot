package preprocess;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import data.Inertial;
import data.InertialAcc;
import data.InertialGyro;
import data.Skeleton;
import data.SkeletonAcc;
import data.SkeletonGyro;

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
	public static double[][] getDisplacement(ArrayList<Skeleton> someoneJointsPos) {
		//(t) Position frames per segment are calculated into (t-1) Distance data per segment ( t=4 as default)
		double [][] displacement = new double[3][someoneJointsPos.size()-1];
		
		for (int i=0; i < someoneJointsPos.size()-1; i++) {
				displacement[0][i] = someoneJointsPos.get(i+1).getRight_wrist()[0] - someoneJointsPos.get(i).getRight_wrist()[0];
				displacement[1][i] = someoneJointsPos.get(i+1).getRight_wrist()[1] - someoneJointsPos.get(i).getRight_wrist()[1];
				displacement[2][i] = someoneJointsPos.get(i+1).getRight_wrist()[2] - someoneJointsPos.get(i).getRight_wrist()[2];
		}
		
		return displacement;
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
	public static double[][] getVelocity(ArrayList<Skeleton> someoneJointsPos, double sampleingRateOfSkn) {
		//(t-1) Distance data per segment are calculated into (t-1) Velocity data per segment ( t=4 as default)
		double [][] displacement = getDisplacement(someoneJointsPos);
		double [][] velocity = new double[3][displacement[0].length];
		
		for (int i = 0; i < displacement[0].length; i++) {
			for (int axis = 0; axis < 3; axis++) {
				velocity[axis][i] = displacement[axis][i] * sampleingRateOfSkn;
			}
		}
		
		return velocity;
	}
	
	/**
	* process joint positions into acceleration
	*   
	* @param: list of Skeleton objects
	* 
	* @return: arrays of acceleration in each axis
	*/
	public static void setAccelerateion(ArrayList<Skeleton> someoneJointsPos, ArrayList<SkeletonAcc> skeletons_acc, int sampleingRateOfSkn, double thresholdSkeletonAcc) {
		//(t-1) Velocity data per segment are calculated into (t-2) Acceleration data per segment ( t=4 as default)
		double[][] velocity = getVelocity(someoneJointsPos, sampleingRateOfSkn);
		double[]acc = new double[3];
		
		for (int i = 0; i < velocity[0].length-1; i++) {
			acc[0] = (velocity[0][i+1] - velocity[0][i]) * sampleingRateOfSkn;
			acc[1] = (velocity[1][i+1] - velocity[1][i]) * sampleingRateOfSkn;
			acc[2]= (velocity[2][i+1] - velocity[2][i]) * sampleingRateOfSkn;
			
			if(Math.abs(acc[0]) < thresholdSkeletonAcc)
				acc[0] = 0;
			if(Math.abs(acc[1]) < thresholdSkeletonAcc)
				acc[1] = 0;
			if(Math.abs(acc[2]) < thresholdSkeletonAcc)
				acc[2] = 0;
			
			skeletons_acc.add(new SkeletonAcc(acc[0], acc[1], acc[2]));
		}
	}
	
	
	/**
	* process joint orientation into rotation
	*   
	* @param: list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* @param: frame data number per segment(0.2sec) , 20*0.2 = 4 as default
	* 
	* @return: arrays of rotation in each axis
	*/
	public static double[][] getRotation(ArrayList<Skeleton> someoneJointOrit) {
		//(t) Position frames per segment are calculated into (t-1) Distance data per segment ( t=4 as default)
		double [][] rotation = new double[3][someoneJointOrit.size()-1];
		
		for (int i=0; i < someoneJointOrit.size()-1; i++) {
			rotation[0][i] = someoneJointOrit.get(i+1).getRight_wrist()[3] - someoneJointOrit.get(i).getRight_wrist()[3];
			rotation[1][i] = someoneJointOrit.get(i+1).getRight_wrist()[4] - someoneJointOrit.get(i).getRight_wrist()[4];
			rotation[2][i] = someoneJointOrit.get(i+1).getRight_wrist()[5] - someoneJointOrit.get(i).getRight_wrist()[5];
		}
		
		return rotation;
	}
	
	/**
	* process joint rotation into anglevelocity
	*   
	* @param: list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* @param: frame data number per segment(0.2sec) , 20*0.2 = 4 as default
	* 
	* @return: arrays of anglevelocity in each axis
	*/
	public static double[][] getAngleVelocity(ArrayList<Skeleton> someoneJointOrit, double sampleingRateOfSkn) {
		//(t-1) Distance data per segment are calculated into (t-1) Velocity data per segment ( t=4 as default)
		double [][] rotation = getRotation(someoneJointOrit);
		double [][] anglevelocity = new double[3][rotation[0].length];
		
		for (int i = 0; i < rotation[0].length; i++) {
			for (int axis = 0; axis < 3; axis++) {
				anglevelocity[axis][i] = rotation[axis][i] * sampleingRateOfSkn;
			}
		}
		
		return anglevelocity;
	}
	
	/**
	* process joint anglevelocity into gyroscope
	*   
	* @param: list of Skeleton objects
	* 
	* @return: arrays of gyroscope in each axis
	*/
	public static void setGyroscope(ArrayList<Skeleton> someoneJointOrit, ArrayList<SkeletonGyro> skeletons_gyro, int sampleingRateOfSkn, double thresholdSkeletonAcc) {
		//(t-1) Velocity data per segment are calculated into (t-2) Acceleration data per segment ( t=4 as default)
		double[][] anglevelocity = getAngleVelocity(someoneJointOrit, sampleingRateOfSkn);
		double[]gyro = new double[3];
		
		for (int i = 0; i < anglevelocity[0].length-1; i++) {
			gyro[0] = (anglevelocity[0][i+1] - anglevelocity[0][i]) * sampleingRateOfSkn;
			gyro[1] = (anglevelocity[1][i+1] - anglevelocity[1][i]) * sampleingRateOfSkn;
			gyro[2]= (anglevelocity[2][i+1] - anglevelocity[2][i]) * sampleingRateOfSkn;
			
			if(Math.abs(gyro[0]) < thresholdSkeletonAcc)
				gyro[0] = 0;
			if(Math.abs(gyro[1]) < thresholdSkeletonAcc)
				gyro[1] = 0;
			if(Math.abs(gyro[2]) < thresholdSkeletonAcc)
				gyro[2] = 0;
			
			skeletons_gyro.add(new SkeletonGyro(gyro[0], gyro[1], gyro[2]));
		}
	}
	
}
