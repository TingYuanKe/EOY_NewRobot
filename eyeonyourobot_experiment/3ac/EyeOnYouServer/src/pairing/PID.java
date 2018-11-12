package pairing;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;

import data.HeadPos;
import data.Inertial;
import data.Skeleton;
import data.TurnList;
import preprocess.BodyExtraction;
import preprocess.Filter;
import preprocess.ProcessTool;
import preprocess.ReadData;
import preprocess.TurnMag;
import scoring.FusionAlgo;

import eyeonyouserver.MainServerSocket;

public class PID {
	/***
	 * @author PID class is transformed from WeiChun's EyeOnYou Demo code.
	 */
	
	public static int collectSeconds = 5;
	public static int sampleingRateOfSkn = 30;
	public static int sampleingRateOfItl = 100;
	
	public static void startPairing() {
		
		int framesOfSkeletonSegment = collectSeconds*sampleingRateOfSkn*8/10; // 13 samples in 1 seconds
		int framesOfInnertialSegment = collectSeconds*sampleingRateOfItl*8/10; // 100 samples in 1 seconds
		String rootDir = "C:/Users/Public/Data";
		ArrayList<String> I_usersName = new ArrayList<String>();
		ArrayList<Integer> S_skeletonsID = new ArrayList<Integer>();
		
		// Read VSFile.cvs to separate different users in VSFile
		S_skeletonsID = BodyExtraction.bodyCount(rootDir + "/KINECTData/VSFile.csv");
		for (int i = 0; i < S_skeletonsID.size(); i++) {
			BodyExtraction.bodyWriter(rootDir + "/KINECTData/VSFile.csv", 
					rootDir + "/KINECTData/VSFile_" + i + ".csv", S_skeletonsID.get(i));
		}
		
		// Read all inertial.txt from each UEs and accumulate file's name into I_userName
		File[] myFileName = Filter.finder(rootDir + "/IMUData/");
		for(int i=0; i < myFileName.length; i++) {
			String tempWithExtension = myFileName[i].getName();
			String temp = tempWithExtension.substring(0, tempWithExtension.lastIndexOf('.'));
			
			// Ignore buffer file to prevent it from seeing as the duplicate I_usersName.
			if(!temp.contains("_buffer")) {
				I_usersName.add(temp);
			}
		}
		
		// Transform inertial data file from .txt to .csv
		for (int i = 0; i < I_usersName.size(); i++) {
			ProcessTool.reformat(rootDir + "/IMUData/" + I_usersName.get(i) + ".txt", rootDir + "/IMUData/" + I_usersName.get(i) + ".csv");
		}
		
		// Read skeleton data and inertial data of each person
		ArrayList<ArrayList<Skeleton>> skeletons_set = new ArrayList<ArrayList<Skeleton>>();
		ArrayList<ArrayList<Inertial>> inertials_set = new ArrayList<ArrayList<Inertial>>();
		for (int i = 0; i < S_skeletonsID.size(); i++) {
			ArrayList<Skeleton> jointsKinect = ReadData.readKinect_smooth(rootDir + "/KINECTData/VSFile_" + i + ".csv");
			skeletons_set.add(jointsKinect);
		}
		for (int i = 0; i < I_usersName.size(); i++) {
			ArrayList<Inertial> jointsIMU = ReadData.readIMU(rootDir + "/IMUData/" + I_usersName.get(i) + ".csv");
			inertials_set.add(jointsIMU);
		}
		
		//Pair skeleton data with users' IDs every 5 seconds
		ArrayList<Double> scores = new ArrayList<Double>();
		System.out.println("*************************");
		for (int i = 0; i < skeletons_set.size(); i++) {
			for (int j = 0; j < inertials_set.size(); j++) {
				ArrayList<Skeleton> sub_skeletons = new ArrayList<Skeleton>(skeletons_set.get(i).subList(0, framesOfSkeletonSegment));
				ArrayList<Inertial> sub_inertials = new ArrayList<Inertial>(inertials_set.get(j).subList(0, framesOfInnertialSegment));
				
				TurnList kinectTurns = TurnMag.genKINECTTurnList(sub_skeletons, sampleingRateOfSkn);
				TurnList imuTurns = TurnMag.genIMUTurnList(sub_inertials, sampleingRateOfItl);
				
				double score = 0;
				score = FusionAlgo.calResult_alg3(kinectTurns, imuTurns);
				scores.add(score);
				System.out.println("Scores: (i=" + i + ", j=" + j + ") -> " + score);
			}
		}
		System.out.println("*************************");
		
		/***
		 * Get the best matching pairing result from each skeleton to each inertial based on scores.
		 * Assign 1 when the high score with strongest confidence happened.
		 * Assign 0; otherwise.
		 */
		int[][] resultMatrix = IDPairing.pairing(scores, S_skeletonsID.size(), I_usersName.size());
		
		// Write pairing results of each frame every 1 seconds
		int[] match = new int[S_skeletonsID.size()];
		for (int i = 0; i < S_skeletonsID.size(); i++) {
			match[i] = -1;
		}
		for (int i = 0; i < skeletons_set.size(); i++) {
			for (int j = 0; j < inertials_set.size(); j++) {
				if (resultMatrix[i][j] == 1) {
					match[i] = j;
				}
			}
		}
		
		try {
			String[] IDCoordinate = new String[S_skeletonsID.size() * 2];
			CSVWriter cw = new CSVWriter(new FileWriter(rootDir + "/KINECTData/result.csv"), ',', CSVWriter.NO_QUOTE_CHARACTER);  // CSVWriter cw = new CSVWriter(new FileWriter(rootDir + "/KINECTData/result.csv", true), ',', CSVWriter.NO_QUOTE_CHARACTER);
//				for (int k = t * framesOfSkeletonSegment; k < (t+1) * framesOfSkeletonSegment; k++) {
				for (int i = 0; i < S_skeletonsID.size(); i++) {
					if (match[i] >= 0) {
						IDCoordinate[i*2] = String.valueOf(S_skeletonsID.get(i));
						IDCoordinate[i*2 + 1] = I_usersName.get(match[i]);
					} else {
						IDCoordinate[i*2] = String.valueOf(S_skeletonsID.get(i));
						IDCoordinate[i*2 + 1] = "Unknown";
					}
				}
				cw.writeNext(IDCoordinate);
//				}
			cw.close();
			// Complete yielding the pairing result.csv and request Kinect to perform tagging profile name.
			MainServerSocket.clientRunPID.requestKinectTagProfile();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
}