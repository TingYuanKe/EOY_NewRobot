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
import data.InertialAcc;
import data.InertialGyro;
import data.Skeleton;
import data.SkeletonAcc;
import data.SkeletonGyro;
import data.TurnList;
import preprocess.BodyExtraction;
import preprocess.Filter;
import preprocess.InertialInfo;
import preprocess.ProcessTool;
import preprocess.ReadData;
import preprocess.SkeletonInfo;
import preprocess.TurnMag;
import scoring.DTWSimilarity;
import scoring.FusionAlgo;

import eyeonyouserver.MainServerSocket;

public class PID {
	/***
	 * @author PID class is transformed from WeiChun's EyeOnYou Demo code.
	 */
	
	// 1. DTW or directly check cross correlation.
	// 2. samplingRateOfSkn should be retrieved by file size.
	
	public static int collectSeconds = 5;
	public static int sampleingRateOfSkn = 0;
	public static int sampleingRateOfItl = 0;
	public static int confidenceOfSimilarity = -1;
	public static double thresholdSD = 0.001;
	
	final static double thresholdSkeletonAcc = 2;
	final static double thresholdSkeletonGyro = 2;
	final static double thresholdInertialAcc = 5;
	final static double thresholdInertialGyro = 5;
//	final static double durationPerSeg = 0.2; // 0.2secs/segment
//	final static int frameSizePerSeg = (int)(sampleingRateOfSkn* durationPerSeg); // 18frames * 0.2secs = 3frames/segment
	
	public static void main(String[] args) throws IOException {
		startPairing();
	}
	
	public static void startPairing() throws IOException {
		
		int framesOfSkeleton = 10000; // 80 samples within 5 seconds
		int framesOfInnertial = 10000; // 500 samples wihtin 5 seconds
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
		ArrayList<ArrayList<SkeletonAcc>> skeletons_acc_set = new ArrayList<ArrayList<SkeletonAcc>>();
		ArrayList<ArrayList<SkeletonGyro>> skeletons_gyro_set = new ArrayList<ArrayList<SkeletonGyro>>();
		ArrayList<ArrayList<Inertial>> inertials_set = new ArrayList<ArrayList<Inertial>>();
		ArrayList<ArrayList<InertialAcc>> inertials_acc_set = new ArrayList<ArrayList<InertialAcc>>();
		ArrayList<ArrayList<InertialGyro>> inertials_gyro_set = new ArrayList<ArrayList<InertialGyro>>();
		for (int i = 0; i < S_skeletonsID.size(); i++) {
			ArrayList<Skeleton> jointsKinect = ReadData.readKinect(rootDir + "/KINECTData/VSFile_" + i + ".csv");
			skeletons_set.add(jointsKinect);
			
			int tempFramesOfSkeleton = ReadData.countFileLine(rootDir + "/KINECTData/VSFile_" + i + ".csv");
			if (tempFramesOfSkeleton > collectSeconds*30*(8.0/10) && framesOfSkeleton > tempFramesOfSkeleton) {
				framesOfSkeleton = tempFramesOfSkeleton;
			}
		}
		sampleingRateOfSkn = framesOfSkeleton / collectSeconds;
		System.out.println("framesOfSkeleton: " + framesOfSkeleton + ", sampleingRateOfSkn: " + sampleingRateOfSkn);
		// Calculate Skeleton Acceleration based on Read Skeleton Position
		for (int i = 0; i < S_skeletonsID.size(); i++) {
			ArrayList<SkeletonAcc> skeletons_acc= new ArrayList<SkeletonAcc>();;
			SkeletonInfo.setAccelerateion(skeletons_set.get(i), skeletons_acc, sampleingRateOfSkn, thresholdSkeletonAcc);
			skeletons_acc_set.add(skeletons_acc);
			
			ArrayList<SkeletonGyro> skeletons_gyro= new ArrayList<SkeletonGyro>();;
			SkeletonInfo.setGyroscope(skeletons_set.get(i), skeletons_gyro, sampleingRateOfSkn, thresholdSkeletonGyro);
			skeletons_gyro_set.add(skeletons_gyro);
		}
		
		
		for (int i = 0; i < I_usersName.size(); i++) {
			ArrayList<Inertial> jointsIMU = ReadData.readIMU(rootDir + "/IMUData/" + I_usersName.get(i) + ".csv", thresholdInertialAcc);
			inertials_set.add(jointsIMU);
			
			int tempFramesOfInnertial = ReadData.countFileLine(rootDir + "/IMUData/" + I_usersName.get(i) + ".csv");
			if (tempFramesOfInnertial > collectSeconds*100*(8.0/10) && framesOfInnertial > tempFramesOfInnertial) {
				framesOfInnertial = tempFramesOfInnertial;
			}
		}
		sampleingRateOfItl = framesOfInnertial / collectSeconds;
		System.out.println("framesOfInnertial: " + framesOfInnertial + ", sampleingRateOfItl: " + sampleingRateOfItl);
		for (int i = 0; i < I_usersName.size(); i++) {
			ArrayList<InertialAcc> inertial_acc= new ArrayList<InertialAcc>();
			InertialInfo.setAcceleration(inertials_set.get(i), inertial_acc, thresholdInertialAcc);
			inertials_acc_set.add(inertial_acc);

			ArrayList<InertialGyro> inertial_gyro= new ArrayList<InertialGyro>();
			InertialInfo.setGyroscope(inertials_set.get(i), inertial_gyro, thresholdInertialGyro);
			inertials_gyro_set.add(inertial_gyro);
		}
		
//		System.out.println("0-inertial");
//		for( int i = 0; i <inertials_set.get(0).size();i++) {
//			System.out.println(inertials_set.get(0).get(i).getAcc()[0]);
//		}
//		System.out.println("0-skeleton");
//		for( int i = 0; i <skeletons_acc_set.get(0).size();i++) {
//			System.out.println(skeletons_acc_set.get(0).get(i).getAccRight_wrist()[0]);
//		}
//		System.out.println("1-inertial");
//		for( int i = 0; i <inertials_set.get(1).size();i++) {
//			System.out.println(inertials_set.get(1).get(i).getAcc()[0]);
//		}
//		System.out.println("1-skeleton");
//		for( int i = 0; i <skeletons_acc_set.get(1).size();i++) {
//			System.out.println(skeletons_acc_set.get(1).get(i).getAccRight_wrist()[0]);
//		}
		
		//Pair skeleton data with users' IDs every 5 seconds
		ArrayList<Double> scores = new ArrayList<Double>();
        System.out.println("*************************");
		for (int i = 0; i < skeletons_acc_set.size(); i++) {
			for (int j = 0; j < inertials_acc_set.size(); j++) {

				double score = 0;
				score = FusionAlgo.calResult_alg3(skeletons_acc_set.get(i), inertials_acc_set.get(j));
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
		int[][] resultMatrix = IDPairing.pairing_ul(scores, S_skeletonsID.size(), I_usersName.size());
		
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
			String[] IDCoordinate = new String[S_skeletonsID.size() * 2 + 1];
			CSVWriter cw = new CSVWriter(new FileWriter(rootDir + "/KINECTData/result.csv"), ',', CSVWriter.NO_QUOTE_CHARACTER);  // CSVWriter cw = new CSVWriter(new FileWriter(rootDir + "/KINECTData/result.csv", true), ',', CSVWriter.NO_QUOTE_CHARACTER);
			if (confidenceOfSimilarity == 1)
				IDCoordinate[0] = "1";
			else if (confidenceOfSimilarity == 0)
				IDCoordinate[0] = "0";
//				for (int k = t * framesOfSkeletonSegment; k < (t+1) * framesOfSkeletonSegment; k++) {
				for (int i = 0; i < S_skeletonsID.size(); i++) {
					if (match[i] >= 0) {
						IDCoordinate[i*2 + 1] = String.valueOf(S_skeletonsID.get(i));
						IDCoordinate[i*2 + 2] = I_usersName.get(match[i]);
					} else {
						IDCoordinate[i*2 + 1] = String.valueOf(S_skeletonsID.get(i));
						IDCoordinate[i*2 + 2] = "Unknown";
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