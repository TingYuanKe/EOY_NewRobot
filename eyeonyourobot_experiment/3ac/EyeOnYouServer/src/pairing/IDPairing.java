package pairing;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;

import data.Inertial;
import data.SIPair;
import data.Skeleton;
import data.TurnList;
import preprocess.ReadData;
import preprocess.TurnMag;
import scoring.FusionAlgo;

public class IDPairing {
	
	private final static String rootDir_k = "J:/Users/Wei/Desktop/Data/KINECTData/Evaluation/Scalibility/";
	private final static String rootDir_i = "J:/Users/Wei/Desktop/Data/IMUData/Evaluation/Scalibility/";
	private final static int nfiles = 330;
	
	private static ArrayList<String> listFilesForFolder(final File folder) {
		ArrayList<String> fileNames = new ArrayList<String>();
		
	    for (final File fileEntry : folder.listFiles()) {
	        if (fileEntry.isDirectory()) {
	            //listFilesForFolder(fileEntry);
	        } else {
	            fileNames.add(fileEntry.getName());
	            //System.out.println(fileEntry.getName());
	        }
	    }
	    
		return fileNames;
	}
	
	/**
	* pick file id randomly without repetition
	*   
	* @param: the number of files
	* 
	* @return: a list of file id
	*/
	private static ArrayList<Integer> randomPick(int cnt) {
		int pickedNum, i = 0;
		ArrayList<Integer> pickedNums = new ArrayList<Integer>();
		
		while (i < cnt) {
			pickedNum = (int)(Math.random()*nfiles);
			if (!pickedNums.contains(pickedNum)) {
				pickedNums.add(pickedNum);
				i++;
			}
		}
		
		return pickedNums;
	}
	
	/**
	* give a pairing map of skeletons and inertial sensors
	*   
	* @param: similarity scores of each combinations
	* @param: the number of skeletons
	* @param: the number of inertial sensors
	* 
	* @return: a map of matching result
	*/
	public static int[][] pairing(ArrayList<Double> scores, int nS, int nI) {
		double[][] scoreMatrix = new double[nS][nI];
		int[][] matchMatrix = new int[nS][nI];
		
		for (int i = 0; i < nS; i++) {
			for (int j = 0; j < nI; j++) {
				scoreMatrix[i][j] = scores.get(i*nI + j);
			}
		}
		
		ArrayList<SIPair> inferedIDs = getInferedIDsList(scoreMatrix);
		boolean[] vmatched = new boolean[nS];
		boolean[] wmatched = new boolean[nI];
		
		for (SIPair inferedid : inferedIDs) {
			if (!vmatched[inferedid.getSid()] && !wmatched[inferedid.getIid()]) {
				/*if (inferedid.getVid() == inferedid.getWid()) {
					System.out.println("Match: " + inferedid.getVid() + "--->" + inferedid.getWid());
				} else {
					System.err.println("Match: " + inferedid.getVid() + "--->" + inferedid.getWid());
				}*/
				vmatched[inferedid.getSid()] = true;
				wmatched[inferedid.getIid()] = true;
				matchMatrix[inferedid.getSid()][inferedid.getIid()] = 1;
			}
		}
		return matchMatrix;
	}
	
	/**
	* give a pairing map of skeletons and inertial sensors with unique level
	*   
	* @param: similarity scores of each combinations
	* @param: the number of skeletons
	* @param: the number of inertial sensors
	* 
	* @return: a map of matching result
	*/
	public static int[][] pairing_ul(ArrayList<Double> scores, int nS, int nI) {
		double[][] scoreMatrix = new double[nS][nI];
		int[][] matchMatrix = new int[nS][nI];
		
		for (int i = 0; i < nS; i++) {
			for (int j = 0; j < nI; j++) {
				scoreMatrix[i][j] = scores.get(i*nI + j);
			}
		}
		
		ArrayList<SIPair> inferedIDs = getInferedIDsList_ul(scoreMatrix);
		boolean[] vmatched = new boolean[nS];
		boolean[] wmatched = new boolean[nI];
		
		for (SIPair inferedid : inferedIDs) {
			if (!vmatched[inferedid.getSid()] && !wmatched[inferedid.getIid()]) {
				/*if (inferedid.getVid() == inferedid.getWid()) {
					System.out.println("Match: " + inferedid.getVid() + "--->" + inferedid.getWid());
				} else {
					System.err.println("Match: " + inferedid.getVid() + "--->" + inferedid.getWid());
				}*/
				vmatched[inferedid.getSid()] = true;
				wmatched[inferedid.getIid()] = true;
				matchMatrix[inferedid.getSid()][inferedid.getIid()] = 1;
			}
		}
		return matchMatrix;
	}
	
	/**
	* generate a list of skeleton and inertial data pairs with their similarity scores
	*   
	* @param: similarity scores of each combinations
	* 
	* @return: a list SIPair objects
	*/
	private static ArrayList<SIPair> getInferedIDsList(double[][] scoreM) {
		int nS = scoreM.length;
		int nI = scoreM[0].length;
		double[] SD = new double[nS];
		ArrayList<SIPair> inferedIDs = new ArrayList<SIPair>();
		
		for (int i = 0; i < nS; i++) {
			SD[i] = 0;
		}
		
		for (int i = 0; i < nS; i++) {
			for (int j = 0; j < nI; j++) {
				inferedIDs.add(new SIPair(i, j, scoreM[i][j], SD[i]));
			}
		}
		
		Collections.sort(inferedIDs);
		/*System.out.println("Sorted");
		for (InferedID id:inferedIDs) {
			System.out.println(id.getVid() + " " + id.getWid() + " " + id.getScore() + " " + id.getUniquev());
		}*/
		
		return inferedIDs;
	}
	
	/**
	* generate a list of skeleton and inertial data pairs with their similarity scores and unique levels
	*   
	* @param: similarity scores of each combinations
	* 
	* @return: a list SIPair objects
	*/
	private static ArrayList<SIPair> getInferedIDsList_ul(double[][] scoreM) {
		int nS = scoreM.length;
		int nI = scoreM[0].length;
		double[] SD = new double[nS];
		ArrayList<SIPair> inferedIDs = new ArrayList<SIPair>();
		
		for (int i = 0; i < nS; i++) {
			SD[i] = calSD(scoreM[i]);
		}
		
		for (int i = 0; i < nS; i++) {
			for (int j = 0; j < nI; j++) {
				inferedIDs.add(new SIPair(i, j, scoreM[i][j], SD[i]));
			}
		}
		
		Collections.sort(inferedIDs);
		/*System.out.println("Sorted");
		for (InferedID id:inferedIDs) {
			System.out.println(id.getVid() + " " + id.getWid() + " " + id.getScore() + " " + id.getUniquev());
		}*/
		
		return inferedIDs;
	}
	
	/**
	* calculate standard deviation
	*   
	* @param: similarity scores of a skeleton data and its combinations with inertial data
	* 
	* @return: a standard deviation value
	*/
	private static double calSD(double[] scores) {
  		double sum = 0, sqsum = 0;
		
		for (int i = 0; i < scores.length; i++) {
			sum += scores[i];
			sqsum += Math.pow(scores[i], 2);
		}
		
		double SD = Math.pow(sqsum/scores.length - Math.pow(sum/scores.length, 2), 0.5);
		
		return SD; 
	}

//	public static void main(String[] args) {
//		
//		// get list of files
//		File folder = new File(rootDir_k);
//		ArrayList<String> kFileNames = listFilesForFolder(folder);
//		/*for(String file : kFileNames) {
//			System.out.println(file);
//		}*/
//		
//		folder = new File(rootDir_i);
//		ArrayList<String> iFileNames = listFilesForFolder(folder);
//		/*for(String file : iFileNames) {
//			System.out.println(file);
//		}*/
//		
//		int round = 1000;
//		int err1 = 0, err2 = 0, err3 = 0, err11 = 0, err22 = 0, err33 = 0;
//		int nS = 8;
//		int nI = 8;
//		for (int r = 0; r < round; r++) {
//			ArrayList<Integer> pickedNums = randomPick(nS);
//			ArrayList<Double> scores1 = new ArrayList<Double>();
//			ArrayList<Double> scores2 = new ArrayList<Double>();
//			ArrayList<Double> scores3 = new ArrayList<Double>();
//		
//			/*for (int i:pickedNums) {
//				System.out.println(i);
//			}*/
//		
//			for (int i = 0; i < nS; i++) {
//				for(int j = 0; j < nI; j++) {
//					//System.out.println(i + " vs. " + j);
//					ArrayList<Skeleton> jointspos = ReadData.readKinect_smooth(rootDir_k + kFileNames.get(pickedNums.get(i)));
//					ArrayList<Inertial> imupoints = ReadData.readIMU(rootDir_i + iFileNames.get(pickedNums.get(j)));
//				
//					TurnList kinectTurns = TurnMag.genKINECTTurnList(jointspos);
//					TurnList imuTurns = TurnMag.genIMUTurnList(imupoints);
//					scores1.add(FusionAlgo.calResult_alg1(kinectTurns, imuTurns));
//					scores2.add(FusionAlgo.calResult_alg2(kinectTurns, imuTurns));
//					scores3.add(FusionAlgo.calResult_alg3(kinectTurns, imuTurns));
//				}
//			}
//		
//			int[][] result1 = pairing(scores1, nS, nI);
//			int[][] result11 = pairing_ul(scores1, nS, nI);
//			int[][] result2 = pairing(scores2, nS, nI);
//			int[][] result22 = pairing_ul(scores2, nS, nI);
//			int[][] result3 = pairing(scores3, nS, nI);
//			int[][] result33 = pairing_ul(scores3, nS, nI);
//			for (int i = 0; i < nS; i++) {
//				for (int j = 0; j < nI; j++) {
//					//System.out.printf("%d ", result3[i][j]);
//					if (i == j && result1[i][j] != 1) {
//						err1 += 1;
//					}
//					
//					if (i == j && result2[i][j] != 1) {
//						err2 += 1;
//					}
//					
//					if (i == j && result3[i][j] != 1) {
//						err3 += 1;
//					}
//					
//					if (i == j && result11[i][j] != 1) {
//						err11 += 1;
//					}
//					
//					if (i == j && result22[i][j] != 1) {
//						err22 += 1;
//					}
//					
//					if (i == j && result33[i][j] != 1) {
//						err33 += 1;
//					}
//				}
//				//System.out.println();
//			}
//		}
//		double errorRate1 = err1/(double)(nS*round);
//		double errorRate2 = err2/(double)(nS*round);
//		double errorRate3 = err3/(double)(nS*round);
//		double errorRate11 = err11/(double)(nS*round);
//		double errorRate22 = err22/(double)(nS*round);
//		double errorRate33 = err33/(double)(nS*round);
//		System.err.println("Error Rate: " + errorRate1 + " " + errorRate2 + " " + errorRate3 + " " + errorRate11 + " " + errorRate22 + " " + errorRate33);
//	
//	}

}
