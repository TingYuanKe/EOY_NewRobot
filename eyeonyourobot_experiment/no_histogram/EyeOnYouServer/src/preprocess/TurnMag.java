package preprocess;

import java.math.BigDecimal;
import java.util.ArrayList;

import data.Inertial;
import data.Skeleton;
import data.TurnList;
import data.MTurn;
import data.STurn;

/**
* This class is used to manage turns.
*
* @author  WeiChun
*/
public class  TurnMag {
	final static double gap = 0.1;
	final static double thresholdDisp = 1;
	final static double thresholdAvgm = 0.2;
	
	/**
	* generate a list of "Move" and "Stop" turns of skeleton data
	*   
	* @param: a list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* 
	* @return: a TurnList object
	*/
	
//	public static TurnList genKINECTTurnList(ArrayList<Skeleton> jointspos, int sampleingRateOfSkn) {
//		final double durationPerSeg = 0.2; // 0.2secs/segment
//		final int frameSizePerSeg = (int)(sampleingRateOfSkn* durationPerSeg); // 18frames * 0.2secs = 3frames/segment
//		
//		double dis[][] = SkeletonInfo.getDistanceList(jointspos, sampleingRateOfSkn, frameSizePerSeg);
//		double[] sum_dis = new double[3];
//		double mag_dis;
//		
//		int idx = 0;
//		ArrayList<Integer> move_seg = new ArrayList<Integer>();
//		ArrayList<MTurn> move_turns = new ArrayList<MTurn>();
//		ArrayList<STurn> still_turns = new ArrayList<STurn>();
//		
//		// move detection and generate move turns
//		for (int i = 0; i < dis[0].length; i+=frameSizePerSeg) {
//			for (int axis = 0; axis < 3; axis++) {
//				double sum = 0;
//				for (int j = 0; j < frameSizePerSeg; j++) {
//					sum += dis[axis][i+j];
//				}
//				sum_dis[axis] = Math.abs(sum) * 100;
//			}
//			mag_dis = Math.pow(Math.pow(sum_dis[0], 2) + Math.pow(sum_dis[1], 2) + Math.pow(sum_dis[2], 2), 0.5);
////			System.out.println("mag_dis: " + mag_dis);
//			//--------------name parameter!!!!!!!!!!!!
//			if (mag_dis >= thresholdDisp) { 
//				move_seg.add(idx);
//			//-----------------------------------------
//			} else {
//				if (move_seg.size() > 1) {
//					double st = new BigDecimal(move_seg.get(0) * durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//					double et = new BigDecimal(move_seg.get(move_seg.size()-1) * durationPerSeg + durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//					MTurn moveturn = new MTurn(st, et);
//					move_turns.add(moveturn);
//				}
//				move_seg.clear();
//			}
//			
//			idx++;
//		}
//		
//		if (move_seg.size() > 1) {
//			double st = new BigDecimal(move_seg.get(0) * durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//			double et = new BigDecimal(move_seg.get(move_seg.size()-1) * durationPerSeg + durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//			MTurn moveturn = new MTurn(st, et);
//			move_turns.add(moveturn);
//		}
//		//----------------name parameter gap!--------------------------------
//		move_turns = mergeMoveTurn(move_turns, gap);
//		
//		double end = new BigDecimal(jointspos.size() / sampleingRateOfSkn).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//		if (move_turns == null) {
//			STurn stillturn = new STurn(0, end);
//			still_turns.add(stillturn);
//		} else {
//			// generate still turns for the very beginning.
//			if (move_turns.get(0).getStartTime() != 0) {
//				STurn stillturn = new STurn(0, move_turns.get(0).getStartTime());
//				still_turns.add(stillturn);
//			}
//			// generate still turns for the middle.
//			for (int i = 0; i < move_turns.size() - 1; i++) {
//				STurn stillturn = new STurn(move_turns.get(i).getEndTime(),
//						move_turns.get(i + 1).getStartTime());
//				still_turns.add(stillturn);
//			}
//			// generate still turns for the very end.
//			if (move_turns.get(move_turns.size() - 1).getEndTime() != end) {
//				STurn stillturn = new STurn(move_turns.get(move_turns.size() - 1).getEndTime(), end);
//				still_turns.add(stillturn);
//			}
//			
//			// add acceleration values to move turns
//			addAcceleration_kinect(move_turns, jointspos, sampleingRateOfSkn, frameSizePerSeg, durationPerSeg);
//		}
//		
//		return new TurnList(still_turns, move_turns);
//		
//	}
//	
//	/**
//	* generate a list of "Move" and "Stop" turns of inertial data
//	*   
//	* @param: a list of Inertia objects
//	* @param: sampling rate for IMU tracking inertial per second (100 as default).
//	* @return: a TurnList object
//	*/
//	public static TurnList genIMUTurnList(ArrayList<Inertial> imu_3, int sampleingRateOfItl) {
//		final double durationPerSeg = 0.2; // 0.2secs/segment
//		final int dataSizePerSeg = (int)(sampleingRateOfItl* durationPerSeg); // 100data * 0.2secs = 20data/segment
//		int idx = 0;
//		ArrayList<STurn> still_turns = new ArrayList<STurn>();
//		ArrayList<Integer> move_seg = new ArrayList<Integer>();
//		ArrayList<MTurn> move_turns = new ArrayList<MTurn>();
//		
//		double sum_accmags, mean_accmags;
//		int c = 0;
//		for (int i = 0; i < imu_3.size(); i+=dataSizePerSeg) {
//			sum_accmags = 0;
//			if (i + dataSizePerSeg >= imu_3.size()) {
//				break;
//			}
//			
//			for (int j = 0; j < dataSizePerSeg; j++) {
//				sum_accmags += Math.pow(Math.pow(imu_3.get(i+j).getAcc()[0], 2) + Math.pow(imu_3.get(i+j).getAcc()[1], 2) + Math.pow(imu_3.get(i+j).getAcc()[2], 2), 0.5);   
//			}
//			mean_accmags = sum_accmags / dataSizePerSeg;
//			
////			System.out.println("mean_accmags: " + mean_accmags);
//			
//			if (mean_accmags >= thresholdAvgm) {
//				move_seg.add(idx);
//			} else {
//				if (move_seg.size() > 1) {
//					//0
//					double st = new BigDecimal(move_seg.get(0) * durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//					double et = new BigDecimal(move_seg.get(move_seg.size()-1) * durationPerSeg + durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//					MTurn moveturn = new MTurn(st, et);
//					move_turns.add(moveturn);
//				}
//				move_seg.clear();
//			}
//			
//			idx++;
//		}
//		
//		// Generate final move turn in case there is no sufficient 10 data sizes.
//		if (move_seg.size() > 1) {
//			double st = new BigDecimal(move_seg.get(0) * durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//			double et = new BigDecimal(move_seg.get(move_seg.size()-1) * durationPerSeg + durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//			MTurn moveturn = new MTurn(st, et);
//			move_turns.add(moveturn);
//		}
//		//--------------------name parameter gap!--------------------------------
//		move_turns = mergeMoveTurn(move_turns, gap);
//		
//		// generate still turns
//		double end = new BigDecimal(imu_3.size() / sampleingRateOfItl).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//		if (move_turns == null) {
//			STurn stillturn = new STurn(0, end);
//			still_turns.add(stillturn);
//		} else {
//			if (move_turns.get(0).getStartTime() != 0) {
//				STurn stillturn = new STurn(0, move_turns.get(0).getStartTime());
//				still_turns.add(stillturn);
//			}
//			for (int i = 0; i < move_turns.size() - 1; i++) {
//				STurn stillturn = new STurn(move_turns.get(i).getEndTime(),
//				move_turns.get(i + 1).getStartTime());
//				still_turns.add(stillturn);
//			}
//
//			if (move_turns.get(move_turns.size() - 1).getEndTime() != end) {
//				STurn stillturn = new STurn(move_turns.get(move_turns.size() - 1).getEndTime(), end);
//				still_turns.add(stillturn);
//			}
//			addAcceleration_imu(move_turns, imu_3, sampleingRateOfItl, dataSizePerSeg, durationPerSeg);
//		}
//		return new TurnList(still_turns, move_turns);
//	
//	}
//	
//	/**
//	* merge "Move" turns with little gap into one turn
//	*   
//	* @param: a list of MTurn objects
//	* @param: gap
//	* 
//	* @return: a list of MTurn objects
//	*/
//	private static ArrayList<MTurn> mergeMoveTurn(ArrayList<MTurn> Turns, double gap) {
//		
//		if(Turns.size() > 0) {
//			ArrayList<MTurn> newTurns = new ArrayList<MTurn>();
//			
//			double startTime = 0, endTime = 0;
//			for(int i = 0, flag = 0; i < Turns.size(); i++) {
//				if (flag == 0) {
//					startTime = Turns.get(i).getStartTime();
//					endTime = Turns.get(i).getEndTime();
//					flag = 1;
//				} else {
//					double tmpGap = new BigDecimal(Turns.get(i).getStartTime() - Turns.get(i-1).getEndTime()).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
//				
//					if(tmpGap <= gap) {
//						endTime = Turns.get(i).getEndTime();
//					} else {
//						newTurns.add(new MTurn(startTime, endTime));
//						flag = 0;
//						i--;
//					}
//				}
//			}
//			newTurns.add(new MTurn(startTime, endTime));
//			
//			return newTurns;
//		} else {
//			return null;
//		}
//		
//	}
//	
	/**
	* add acceleration values to each "Move" turn which processed from skeleton data
	*   
	* @param: a list of MTurn objects
	* @param: a list of Skeleton objects
	* @param: sampling rate for Kinect tracking skeleton per second (fps).
	* @param: frame data number per segment(0.2sec) , 20*0.2 = 4 as default
	*/
//	public static void addAcceleration_kinect(ArrayList<Skeleton> jointspos, int sampleingRateOfSkn, int frameSizePerSeg, double durationPerSeg) {
//		ArrayList<Skeleton> certainMove;
//		double[][] acc;
//		
//			
//			acc = SkeletonInfo.getAccList(jointspos, sampleingRateOfSkn, frameSizePerSeg);
////			for (int j = 0; j < acc[0].length; j++) {
////				Turns.get(i).addAcc(Math.pow(Math.pow(acc[0][j], 2) + Math.pow(acc[1][j], 2) + Math.pow(acc[2][j], 2), 0.5));
////				Turns.get(i).addAcc_x(acc[0][j]);
////				Turns.get(i).addAcc_y(acc[1][j]);
////				Turns.get(i).addAcc_z(acc[2][j]);
////			}
//		
//		
//	}
	
	/**
	* add acceleration values to each "Move" turn which processed from inertial data
	*   
	* @param: a list of MTurn objects
	* @param: a list of inertia objects
	*/
//	private static void addAcceleration_imu(ArrayList<MTurn> Turns, ArrayList<Inertial> imu_6, int sampleingRateOfItl, int dataSizePerSeg, double durationPerSeg) {
//		ArrayList<Inertial> imu_move;
//		for (int i = 0; i < Turns.size(); i++) {
//			int head = new BigDecimal(Turns.get(i).getStartTime() / durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).intValue();
//			int tail = new BigDecimal(Turns.get(i).getEndTime() / durationPerSeg).setScale(1, BigDecimal.ROUND_HALF_DOWN).intValue();
//			imu_move = new ArrayList<Inertial>(imu_6.subList(head * dataSizePerSeg, tail * dataSizePerSeg));
//			calAcceleration(Turns.get(i), imu_move);
//		}
//		
//	}
	
	/**
	* downsample acceleration values in each "Move" turn which processed from inertial data
	*   
	* @param: a MTurn object
	* @param: a list of inertia objects
	*/
//	private static void calAcceleration(MTurn MoveTurn, ArrayList<Inertial> imu_move) {
//		double tmpAcc = 0, tmpAccX = 0, tmpAccY = 0, tmpAccZ = 0;
//		int cnt = 0;
//		
//		for (Inertial imu:imu_move) {
//			tmpAcc += Math.pow(Math.pow(imu.getAcc()[0], 2) + Math.pow(imu.getAcc()[1], 2) + Math.pow(imu.getAcc()[2], 2), 0.5);
//			tmpAccX += imu.getAcc()[0];
//			tmpAccY += imu.getAcc()[1];
//			tmpAccZ += imu.getAcc()[2];
//			cnt++;
//			if (cnt == 5) {
//				MoveTurn.addAcc(tmpAcc/5);
//				MoveTurn.addAcc_x(tmpAccX/5);
//				MoveTurn.addAcc_y(tmpAccY/5);
//				MoveTurn.addAcc_z(tmpAccZ/5);
//				tmpAcc = 0;
//				tmpAccX = 0;
//				tmpAccY = 0;
//				tmpAccZ = 0;
//				cnt = 0;
//			}
//		}
//		if (cnt != 0) {
//			MoveTurn.addAcc(tmpAcc/cnt);
//			MoveTurn.addAcc_x(tmpAccX/cnt);
//			MoveTurn.addAcc_y(tmpAccY/cnt);
//			MoveTurn.addAcc_z(tmpAccZ/cnt);
//		}
//	}

}
