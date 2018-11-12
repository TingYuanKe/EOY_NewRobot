package scoring;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Arrays;

import data.Turn;
import data.TurnList;
import data.MTurn;
import data.STurn;

/**
 * This class is used to calculate similarity score with a certain fusion
 * algorithm.
 *
 * @author WeiChun
 */
public class FusionAlgo {

	/**
	 * calculate similarity score with "SM" algorithm
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: similarity score
	 */
	public static double calResult_alg1(TurnList fromVideo, TurnList fromIMU) {
		ArrayList<STurn> stillturnsQueue = genStillTurnsQueue(fromVideo, fromIMU);
		ArrayList<MTurn> moveturnsQueue = genMoveTurnsQueue(fromVideo, fromIMU);

		// System.out.println(stillturnsQueue.size() + " - " + moveturnsQueue.size());

		ArrayList<Turn> mixedturnsQueue_vMiS = genMixedTurnsQueue_vMiS(fromVideo, fromIMU);
		ArrayList<Turn> mixedturnsQueue_vSiM = genMixedTurnsQueue_vSiM(fromVideo, fromIMU);

		// System.out.println(mixedturnsQueue_vMiS.size() + " - " +
		// mixedturnsQueue_vSiM.size());

		double[] bonus = calBonus(stillturnsQueue, moveturnsQueue);
		double[] penalty = calPenalty(mixedturnsQueue_vMiS, mixedturnsQueue_vSiM);
		double result = bonus[0] + penalty[0];
		double resultCnt = bonus[1] + penalty[1];
		// System.out.println("Result: " + result + " Cnt: " + resultCnt + " Score: " +
		// result / resultCnt);

		return (result / resultCnt);
	}

	/**
	 * calculate similarity score with AC algorithm
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: similarity score
	 */
	public static double calResult_alg2(TurnList fromVideo, TurnList fromIMU) {
		double[][] KinectAccSeq = getAccSeq(fromVideo);
		double[][] IMUAccSeq = getAccSeq(fromIMU);
		int lag = 40;

		for (int i = 0; i < 4; i++) {
			IMUAccSeq[i] = Arrays.copyOf(IMUAccSeq[i], KinectAccSeq[i].length);
		}

		double[] xcorr = calXCorr(KinectAccSeq[0], IMUAccSeq[0], lag);
		double max = 0;
		for (int i = 0; i < 2 * lag + 1; i++) {
			max = (xcorr[i] > max) ? xcorr[i] : max;
		}

		return max;

	}

	/**
	 * calculate similarity score with 3AC algorithm
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: similarity score
	 */
	public static double calResult_alg3(TurnList fromVideo, TurnList fromIMU) {
		double[][] KinectAccSeq = getAccSeq(fromVideo);
		double[][] IMUAccSeq = getAccSeq(fromIMU);
		double[][] xcorr = new double[3][];
		int lag = 40;

		System.out.println("KinectAccSeq");
		for (int i = 0; i < KinectAccSeq[0].length; i++) {
			System.out.println( KinectAccSeq[0][i] );
		}
		System.out.println("IMUAccSeq");
		for (int i = 0; i < IMUAccSeq[0].length; i++) {
			System.out.println( IMUAccSeq[0][i] );
		}
		
		for (int i = 0; i < 4; i++) {
			IMUAccSeq[i] = Arrays.copyOf(IMUAccSeq[i], KinectAccSeq[i].length);
		}

		xcorr[0] = calXCorr(KinectAccSeq[1], IMUAccSeq[2], lag);
		xcorr[1] = calXCorr(KinectAccSeq[2], IMUAccSeq[3], lag);
		xcorr[2] = calXCorr(KinectAccSeq[3], IMUAccSeq[1], lag);
		/*
		 * for (int i = 0; i < xcorr[0].length; i++) { System.out.println(xcorr[0][i]);
		 * }
		 */

		/*
		 * for (int i = 0; i < KinectAccSeq[0].length; i++) { System.out.printf("%f ",
		 * KinectAccSeq[2][i]); } System.out.println("");
		 * 
		 * for (int i = 0; i < IMUAccSeq[0].length; i++) { System.out.printf("%f ",
		 * IMUAccSeq[3][i]); } System.out.println("");
		 */

		double max = 0;
		for (int i = 0; i < 2 * lag + 1; i++) {
			// System.out.println(xcorr[2][i]);
			double tmpXCorr = xcorr[0][i] + xcorr[1][i] + xcorr[2][i];
			max = (tmpXCorr > max) ? tmpXCorr : max;
		}
		/*
		 * for (int i = 0; i < 3; i++) { double tmpXCorr = -1; for (int j = 0; j < 2 *
		 * lag + 1; j++) { tmpXCorr = (tmpXCorr > xcorr[i][j])?tmpXCorr:xcorr[i][j]; }
		 * max += tmpXCorr; }
		 */

		return max;

	}

	/**
	 * Generate queue of "Stop" turns from skeleton data and inertial data. The list
	 * will be used to calculate S-S overlaps
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: list of "Stop" turns
	 */
	private static ArrayList<STurn> genStillTurnsQueue(TurnList fromVideo, TurnList fromIMU) {
		ArrayList<STurn> kinectStillTurns = fromVideo.getStillTurnList();
		ArrayList<STurn> imuStillTurns = fromIMU.getStillTurnList();
		ArrayList<STurn> stillturnsQueue = new ArrayList<STurn>();
		int i = 0, j = 0;

		if (kinectStillTurns == null && imuStillTurns != null) {
			for (STurn turn : imuStillTurns) {
				stillturnsQueue.add(turn);
			}
		} else if (imuStillTurns == null && kinectStillTurns != null) {
			for (STurn turn : kinectStillTurns) {
				stillturnsQueue.add(turn);
			}
		} else if (imuStillTurns != null && kinectStillTurns != null) {
			while (i < kinectStillTurns.size() && j < imuStillTurns.size()) {
				if (kinectStillTurns.get(i).getStartTime() <= imuStillTurns.get(j).getStartTime()) {
					stillturnsQueue.add(kinectStillTurns.get(i));
					i++;
				} else {
					stillturnsQueue.add(imuStillTurns.get(j));
					j++;
				}
			}
			if (i == kinectStillTurns.size()) {
				stillturnsQueue.add(imuStillTurns.get(j));
			} else {
				stillturnsQueue.add(kinectStillTurns.get(i));
			}
		}

		return stillturnsQueue;

	}

	/**
	 * Generate queue of "Move" turns from skeleton data and inertial data. The list
	 * will be used to calculate M-M overlaps
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: list of "Move" turns
	 */
	private static ArrayList<MTurn> genMoveTurnsQueue(TurnList fromVideo, TurnList fromIMU) {
		ArrayList<MTurn> kinectMoveTurns = fromVideo.getMoveTurnList();
		ArrayList<MTurn> imuMoveTurns = fromIMU.getMoveTurnList();
		ArrayList<MTurn> MoveturnsQueue = new ArrayList<MTurn>();
		int i = 0, j = 0;

		if (kinectMoveTurns == null && imuMoveTurns != null) {
			for (MTurn turn : imuMoveTurns) {
				MoveturnsQueue.add(turn);
			}
		} else if (imuMoveTurns == null && kinectMoveTurns != null) {
			for (MTurn turn : kinectMoveTurns) {
				MoveturnsQueue.add(turn);
			}
		} else if (kinectMoveTurns != null && imuMoveTurns != null) {
			while (i < kinectMoveTurns.size() && j < imuMoveTurns.size()) {
				if (kinectMoveTurns.get(i).getStartTime() <= imuMoveTurns.get(j).getStartTime()) {
					MoveturnsQueue.add(kinectMoveTurns.get(i));
					i++;
				} else {
					MoveturnsQueue.add(imuMoveTurns.get(j));
					j++;
				}
			}
			if (i == kinectMoveTurns.size()) {
				MoveturnsQueue.add(imuMoveTurns.get(j));
			} else {
				MoveturnsQueue.add(kinectMoveTurns.get(i));
			}
		}

		return MoveturnsQueue;

	}

	/**
	 * Generate mixed queue of "Move" turns from skeleton data "Stop" turns from
	 * inertial data. The list will be used to calculate M-S overlaps
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: list of M-S turns
	 */
	private static ArrayList<Turn> genMixedTurnsQueue_vMiS(TurnList fromVideo, TurnList fromIMU) {
		ArrayList<MTurn> kinectMoveTurns = fromVideo.getMoveTurnList();
		ArrayList<STurn> imuStillTurns = fromIMU.getStillTurnList();
		ArrayList<Turn> mixedturnsQueue = new ArrayList<Turn>();
		int i = 0, j = 0;

		if (kinectMoveTurns == null && imuStillTurns != null) {
			for (STurn turn : imuStillTurns) {
				mixedturnsQueue.add(turn);
			}
		} else if (kinectMoveTurns != null && imuStillTurns == null) {
			for (MTurn turn : kinectMoveTurns) {
				mixedturnsQueue.add(turn);
			}
		} else if (kinectMoveTurns != null && imuStillTurns != null) {
			while (i < kinectMoveTurns.size() && j < imuStillTurns.size()) {
				if (kinectMoveTurns.get(i).getStartTime() <= imuStillTurns.get(j).getStartTime()) {
					mixedturnsQueue.add(kinectMoveTurns.get(i));
					i++;
				} else {
					mixedturnsQueue.add(imuStillTurns.get(j));
					j++;
				}
			}
			if (i == kinectMoveTurns.size()) {
				mixedturnsQueue.add(imuStillTurns.get(j));
			} else {
				mixedturnsQueue.add(kinectMoveTurns.get(i));
			}
		}
		return mixedturnsQueue;

	}

	/**
	 * Generate mixed queue of "Stop" turns from skeleton data "Move" turns from
	 * inertial data. The list will be used to calculate S-M overlaps
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: list of S-M turns
	 */
	private static ArrayList<Turn> genMixedTurnsQueue_vSiM(TurnList fromVideo, TurnList fromIMU) {
		ArrayList<STurn> kinectStillTurns = fromVideo.getStillTurnList();
		ArrayList<MTurn> imuMoveTurns = fromIMU.getMoveTurnList();
		ArrayList<Turn> mixedturnsQueue = new ArrayList<Turn>();
		int i = 0, j = 0;

		if (kinectStillTurns == null && imuMoveTurns != null) {
			for (MTurn turn : imuMoveTurns) {
				mixedturnsQueue.add(turn);
			}
		} else if (kinectStillTurns != null && imuMoveTurns == null) {
			for (STurn turn : kinectStillTurns) {
				mixedturnsQueue.add(turn);
			}
		} else if (kinectStillTurns != null && imuMoveTurns != null) {
			while (i < kinectStillTurns.size() && j < imuMoveTurns.size()) {
				if (kinectStillTurns.get(i).getStartTime() <= imuMoveTurns.get(j).getStartTime()) {
					mixedturnsQueue.add(kinectStillTurns.get(i));
					i++;
				} else {
					mixedturnsQueue.add(imuMoveTurns.get(j));
					j++;
				}
			}
			if (i == kinectStillTurns.size()) {
				mixedturnsQueue.add(imuMoveTurns.get(j));
			} else {
				mixedturnsQueue.add(kinectStillTurns.get(i));
			}
		}

		return mixedturnsQueue;

	}

	/**
	 * Generate mixed queue of "Stop" turns from skeleton data "Move" turns from
	 * inertial data. The list will be used to calculate S-M overlaps
	 * 
	 * @param: turn
	 *             recorded form skeleton data
	 * @param: turn
	 *             recorded form inertial data
	 * 
	 * @return: list of S-M turns
	 */
	private static double[] calBonus(ArrayList<STurn> stillturnsQueue, ArrayList<MTurn> moveturnsQueue) {
		double bonus = 0;
		int bonusCnt = 0;

		BonusFunction bf = new BonusFunction(20, 2);

		for (int i = 0; i < stillturnsQueue.size() - 1; i++) {
			double overlap = Math.min(stillturnsQueue.get(i).getEndTime() - stillturnsQueue.get(i + 1).getStartTime(),
					stillturnsQueue.get(i + 1).getEndTime() - stillturnsQueue.get(i + 1).getStartTime());
			overlap = new BigDecimal(overlap).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
			if (overlap >= 0) {
				bonus += bf.getBonusValue(overlap);
				// System.out.println("Bonus: " + overlap + " - " + bf.getBonusValue(overlap));
				bonusCnt += 1;
			}
		}

		BonusFunction bf_m = new BonusFunction(30, 1);

		for (int i = 0; i < moveturnsQueue.size() - 1; i++) {
			double overlap = Math.min(moveturnsQueue.get(i).getEndTime() - moveturnsQueue.get(i + 1).getStartTime(),
					moveturnsQueue.get(i + 1).getEndTime() - moveturnsQueue.get(i + 1).getStartTime());
			overlap = new BigDecimal(overlap).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
			if (overlap >= 0) {
				bonus += bf_m.getBonusValue(overlap);
				// System.out.println("Bonus: " + overlap + " - " + bf.getBonusValue(overlap));
				bonusCnt += 1;
			}
		}

		return new double[] { bonus, bonusCnt };

	}

	private static double[] calPenalty(ArrayList<Turn> mixedturnsQueue_vMiS, ArrayList<Turn> mixedturnsQueue_vSiM) {
		double penalty = 0;
		int penaltyCnt = 0;

		PenaltyFunction bf = new PenaltyFunction(2, 2);

		for (int i = 0; i < mixedturnsQueue_vMiS.size() - 1; i++) {
			double overlap = Math.min(
					mixedturnsQueue_vMiS.get(i).getEndTime() - mixedturnsQueue_vMiS.get(i + 1).getStartTime(),
					mixedturnsQueue_vMiS.get(i + 1).getEndTime() - mixedturnsQueue_vMiS.get(i + 1).getStartTime());
			overlap = new BigDecimal(overlap).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
			if (overlap > 0) {
				penalty += bf.getPenaltyValue(overlap);
				// System.out.println("Penalty: " + overlap + " - " +
				// bf.getPenaltyValue(overlap));
				penaltyCnt += 1;
			}
		}

		for (int i = 0; i < mixedturnsQueue_vSiM.size() - 1; i++) {
			double overlap = Math.min(
					mixedturnsQueue_vSiM.get(i).getEndTime() - mixedturnsQueue_vSiM.get(i + 1).getStartTime(),
					mixedturnsQueue_vSiM.get(i + 1).getEndTime() - mixedturnsQueue_vSiM.get(i + 1).getStartTime());
			overlap = new BigDecimal(overlap).setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
			if (overlap > 0) {
				penalty += bf.getPenaltyValue(overlap);
				// System.out.println("Penalty: " + overlap + " - " +
				// bf.getPenaltyValue(overlap));
				penaltyCnt += 1;
			}
		}

		return new double[] { penalty, penaltyCnt };
	}

	private static double[][] getAccSeq(TurnList turnsList) {
		ArrayList<MTurn> MTurns = turnsList.getMoveTurnList();
		ArrayList<STurn> STurns = turnsList.getStillTurnList();
		ArrayList<Double> acc = new ArrayList<Double>();
		ArrayList<Double> accX = new ArrayList<Double>();
		ArrayList<Double> accY = new ArrayList<Double>();
		ArrayList<Double> accZ = new ArrayList<Double>();
		int i = 0, j = 0, time;

		// When there are all Move Turns without any Stop Turn
		if (MTurns != null && STurns == null) {
			for (int k = 0; k < MTurns.size(); k++) {
				for (Double stepAcc : MTurns.get(j).getAcc()) {
					acc.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_x()) {
					accX.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_y()) {
					accY.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_z()) {
					accZ.add(stepAcc);
				}
			}
		} 
		// When there are all Stop Turns without any Move Turn
		else if (MTurns == null && STurns != null) {
			for (int k = 0; k < STurns.size(); k++) {
				double dur = new BigDecimal(STurns.get(i).getEndTime() - STurns.get(i).getStartTime())
						.setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
				time = new BigDecimal(dur / 0.1).setScale(0, BigDecimal.ROUND_HALF_DOWN).intValue();
				for (int t = 0; t < time; t++) {
					acc.add(0.0);
					accX.add(0.0);
					accY.add(0.0);
					accZ.add(0.0);
				}
			}
		}
		// When there are the combinations of Move Turns and Stop Turns
		else if (MTurns != null && STurns != null) {
			while (true) {
				if (i < STurns.size() && j < MTurns.size()
						&& STurns.get(i).getStartTime() < MTurns.get(j).getStartTime()) {
					double dur = new BigDecimal(STurns.get(i).getEndTime() - STurns.get(i).getStartTime())
							.setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
					time = new BigDecimal(dur / 0.1).setScale(0, BigDecimal.ROUND_HALF_DOWN).intValue();
					for (int t = 0; t < time; t++) {
						acc.add(0.0);
						accX.add(0.0);
						accY.add(0.0);
						accZ.add(0.0);
					}
					i += 1;
				} else if (i < STurns.size() && j < MTurns.size()
						&& MTurns.get(j).getStartTime() < STurns.get(i).getStartTime()) {
					for (Double stepAcc : MTurns.get(j).getAcc()) {
						acc.add(stepAcc);
					}
					for (Double stepAcc : MTurns.get(j).getAcc_x()) {
						accX.add(stepAcc);
					}
					for (Double stepAcc : MTurns.get(j).getAcc_y()) {
						accY.add(stepAcc);
					}
					for (Double stepAcc : MTurns.get(j).getAcc_z()) {
						accZ.add(stepAcc);
					}
					j += 1;
				}

				if (i == STurns.size() || j == MTurns.size()) {
					break;
				}

			}

			while (i < STurns.size()) {
				double dur = new BigDecimal(STurns.get(i).getEndTime() - STurns.get(i).getStartTime())
						.setScale(1, BigDecimal.ROUND_HALF_DOWN).doubleValue();
				time = new BigDecimal(dur / 0.1).setScale(1, BigDecimal.ROUND_HALF_DOWN).intValue();
				for (int t = 0; t < time; t++) {
					acc.add(0.0);
					accX.add(0.0);
					accY.add(0.0);
					accZ.add(0.0);
				}
				i += 1;
			}

			while (j < MTurns.size()) {
				for (Double stepAcc : MTurns.get(j).getAcc()) {
					acc.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_x()) {
					accX.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_y()) {
					accY.add(stepAcc);
				}
				for (Double stepAcc : MTurns.get(j).getAcc_z()) {
					accZ.add(stepAcc);
				}
				j += 1;
			}
		}

		double[][] AccSeq = new double[4][acc.size()];
		for (int k = 0; k < acc.size(); k++) {
			AccSeq[0][k] = acc.get(k);
			AccSeq[1][k] = accX.get(k);
			AccSeq[2][k] = accY.get(k);
			AccSeq[3][k] = accZ.get(k);
		}

		return AccSeq;

	}

	/*
	 * private static double calPCC(double[] kAccSeq, double[] iAccSeq) { double
	 * num, den, den1, den2; double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0,
	 * sum_yy = 0; int len = (kAccSeq.length < iAccSeq.length)? kAccSeq.length :
	 * iAccSeq.length;
	 * 
	 * for (int i = 0; i < len; i++) { sum_xy += kAccSeq[i] * iAccSeq[i]; sum_x +=
	 * kAccSeq[i]; sum_y += iAccSeq[i]; sum_xx += kAccSeq[i] * kAccSeq[i]; sum_yy +=
	 * iAccSeq[i] * iAccSeq[i]; }
	 * 
	 * num = (len * sum_xy) - (sum_x * sum_y); den1 = (len * sum_xx) - (sum_x *
	 * sum_x); den2 = (len * sum_yy) - (sum_y * sum_y); den = Math.pow(den1, 0.5) *
	 * Math.pow(den2, 0.5);
	 * 
	 * return num/den;
	 * 
	 * }
	 */

	private static double[] calXCorr(double[] kAccSeq, double[] iAccSeq, int lag) {
		double[] xcorr = new double[lag * 2 + 1];
		int idx = 0;

		// coeff normalization
		double nomalizer = 0, autoXCorr_k = 0, autoXCorr_i = 0;
		for (int i = 0; i < kAccSeq.length; i++) {
			autoXCorr_k += kAccSeq[i] * kAccSeq[i];
			autoXCorr_i += iAccSeq[i] * iAccSeq[i];
		}
		nomalizer = Math.pow(autoXCorr_k * autoXCorr_i, 0.5);
		nomalizer = (nomalizer != 0) ? nomalizer : 1;

		for (int shift = -lag; shift <= 0; shift++) {
			for (int i = 0; i < kAccSeq.length + shift; i++) {
				xcorr[idx] += kAccSeq[i] * iAccSeq[i + (-shift)];
			}
			xcorr[idx] /= nomalizer;
			idx++;
		}

		for (int shift = 1; shift <= lag; shift++) {
			for (int i = 0; i < kAccSeq.length - shift; i++) {
				xcorr[idx] += kAccSeq[i + shift] * iAccSeq[i];
			}
			xcorr[idx] /= nomalizer;
			idx++;
		}

		return xcorr;
	}

}
