/*package pairing;

import java.util.ArrayList;

import data.Inertial;
import data.Skeleton;
import data.TurnList;
import preprocess.ReadData;
import preprocess.TurnMag;
import data.MTurn;
import data.STurn;
import scoring.FusionAlgo;

public class TurnDetection {

	public static void main(String[] args) {
		String rootDir = "J:/Users/Wei/Desktop/Data";
		String imu_acc_files = "circle2_acc.csv";
		String video_files = "circle2.csv";

		ArrayList<Skeleton> jointspos = ReadData.readKinect_smooth(rootDir + "/KINECTData/Evaluation/abs/" + video_files);
		ArrayList<Inertial> imu_6 = ReadData.readIMU(rootDir + "/IMUData/Evaluation/absacc/" + imu_acc_files);

		System.out.println(imu_6.size() + " - " + jointspos.size());

		TurnList imuTurns = TurnMag.genIMUTurnList(imu_6);
		TurnList kinectTurns = TurnMag.genKINECTTurnList(jointspos);

		if (kinectTurns.getMoveTurnList() != null) {
			System.out.println("Move: " + kinectTurns.getMoveTurnList().size());
			for (MTurn turn : kinectTurns.getMoveTurnList()) {
				System.out.println(turn.getStartTime() + " - " + turn.getEndTime());
			}
		}
		
		if (kinectTurns.getStillTurnList() != null) {
			System.out.println("Still: " + kinectTurns.getStillTurnList().size());
			for (STurn turn : kinectTurns.getStillTurnList()) {
				System.out.println(turn.getStartTime() + " - " + turn.getEndTime());
			}
		}
		
		System.out.println("======");
		
		if (imuTurns.getMoveTurnList() != null) {
			System.out.println("Move: " + imuTurns.getMoveTurnList().size());
			for (MTurn turn : imuTurns.getMoveTurnList()) {
				System.out.println(turn.getStartTime() + " - " + turn.getEndTime());
			}
		}

		if (imuTurns.getStillTurnList() != null) {
			System.out.println("Still: " + imuTurns.getStillTurnList().size());
			for (STurn turn : imuTurns.getStillTurnList()) {
				System.out.println(turn.getStartTime() + " - " + turn.getEndTime());
			}
		}
		
		System.out.println(FusionAlgo.calResult_alg3(kinectTurns, imuTurns));
		System.out.println("========");
	}

}
*/