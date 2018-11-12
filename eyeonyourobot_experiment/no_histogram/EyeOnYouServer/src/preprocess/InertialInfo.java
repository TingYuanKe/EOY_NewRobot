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

public class InertialInfo {

	/**
	* process joint anglevelocity into gyroscope
	*   
	* @param: list of Skeleton objects
	* 
	* @return: arrays of gyroscope in each axis
	*/
	
	public static void setAcceleration(ArrayList<Inertial> someoneJointPos, ArrayList<InertialAcc> inertial_acc, double thresholdInertialAcc) {
		//(t-1) Velocity data per segment are calculated into (t-2) Acceleration data per segment ( t=4 as default)
		double[]acc = new double[3];
		
		for (int i = 0; i < someoneJointPos.size(); i++) {
			acc[0] = someoneJointPos.get(i).getRight_wrist()[0];
			acc[1] = someoneJointPos.get(i).getRight_wrist()[1];
			acc[2] = someoneJointPos.get(i).getRight_wrist()[2];
			
			if(Math.abs(acc[0]) < thresholdInertialAcc)
				acc[0] = 0;
			if(Math.abs(acc[1]) < thresholdInertialAcc)
				acc[1] = 0;
			if(Math.abs(acc[2]) < thresholdInertialAcc)
				acc[2] = 0;
			
			inertial_acc.add(new InertialAcc(acc[0], acc[1], acc[2]));
		}
	}
	
	
	public static void setGyroscope(ArrayList<Inertial> someoneJointOrit, ArrayList<InertialGyro> inertial_gyro, double thresholdInertialGyro) {
		//(t-1) Velocity data per segment are calculated into (t-2) Acceleration data per segment ( t=4 as default)
		double[]gyro = new double[3];
		
		for (int i = 0; i < someoneJointOrit.size(); i++) {
			gyro[0] = someoneJointOrit.get(i).getRight_wrist()[3];
			gyro[1] = someoneJointOrit.get(i).getRight_wrist()[4];
			gyro[2] = someoneJointOrit.get(i).getRight_wrist()[5];
			
			if(Math.abs(gyro[0]) < thresholdInertialGyro)
				gyro[0] = 0;
			if(Math.abs(gyro[1]) < thresholdInertialGyro)
				gyro[1] = 0;
			if(Math.abs(gyro[2]) < thresholdInertialGyro)
				gyro[2] = 0;
			
			inertial_gyro.add(new InertialGyro(gyro[0], gyro[1], gyro[2]));
		}
	}
	
}
