//package pairing;
//
//import java.io.File;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.ArrayList;
//
//import com.opencsv.CSVWriter;
//
//import data.Inertial;
//import data.Skeleton;
//import data.TurnList;
//import preprocess.ReadData;
//import preprocess.SkeletonInfo;
//import preprocess.TurnMag;
//import data.MTurn;
//import data.STurn;
//
//public class MovementDetection {
//	
//	private final static String rootDir_k = "J:/Users/Wei/Desktop/Data/KINECTData/Evaluation/Continuity/";
//	private final static String rootDir_i = "J:/Users/Wei/Desktop/Data/IMUData/Evaluation/Continuity/";
//	
//	private static ArrayList<String> listFilesForFolder(final File folder) {
//		ArrayList<String> fileNames = new ArrayList<String>();
//		
//	    for (final File fileEntry : folder.listFiles()) {
//	        if (fileEntry.isDirectory()) {
//	            //listFilesForFolder(fileEntry);
//	        } else {
//	            fileNames.add(fileEntry.getName());
//	        }
//	    }
//	    
//		return fileNames;
//	}
//	
//	/*private static void getDisFile(ArrayList<KINECTPoint> jointpos, String file) {
//		double[][] dis = SkeletonInfo.getDistanceList(jointpos);
//		
//		try {
//			CSVWriter cw = new CSVWriter(new FileWriter(rootDir_k + "dis_" + file), ',', CSVWriter.NO_QUOTE_CHARACTER);
//			double[] sum_dis = new double[3];
//			double mag_dis;
//			for (int i = 0; i < dis[0].length; i+=5) {
//				for (int axis = 0; axis < 3; axis++) {
//					double sum = 0;
//					for (int j = 0; j < 5; j++) {
//						sum += dis[axis][i+j];
//					}
//					sum_dis[axis] = Math.abs(sum) * 100;
//				}
//				mag_dis = Math.pow(Math.pow(sum_dis[0], 2) + Math.pow(sum_dis[1], 2) + Math.pow(sum_dis[2], 2), 0.5);
//				cw.writeNext(new String[] { String.valueOf(sum_dis[0]), String.valueOf(sum_dis[1]), String.valueOf(sum_dis[2]), String.valueOf(mag_dis) });
//			}
//			cw.close();
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//	}*/
//
//	public static void main(String[] args) {
//		//final File folder = new File(rootDir_k);
//		File folder = new File(rootDir_k);
//		ArrayList<String> fileNames = listFilesForFolder(folder);
//		
//		for(String filename:fileNames) {
//			System.out.println(filename);
//			ArrayList<Skeleton> jointspos = ReadData.readKinect_smooth(rootDir_k + filename);
//			//getDisFile(jointspos, filename);
//			
//			TurnList kTurns = TurnMag.genKINECTTurnList(jointspos);
//			ArrayList<MTurn> kmTurns = kTurns.getMoveTurnList();
//			ArrayList<STurn> ksTurns = kTurns.getStillTurnList();
//			if (ksTurns != null) {
//				System.out.println("Still:" + ksTurns.size());
//			}
//			
//			if (kmTurns != null) {
//				System.out.println("Move:" + kmTurns.size());
//			}
//		}
//		
//		folder = new File(rootDir_i);
//		fileNames = listFilesForFolder(folder);
//		
//		
//		for(String filename:fileNames) {
//			System.out.println(filename);
//			ArrayList<Inertial> imupoints = ReadData.readIMU(rootDir_i + filename);
//			
//			TurnList iTurns = TurnMag.genIMUTurnList(imupoints);
//			ArrayList<MTurn> imTurns = iTurns.getMoveTurnList();
//			ArrayList<STurn> isTurns = iTurns.getStillTurnList();
//			
//			if (isTurns != null) {
//				System.out.println("Still:" + isTurns.size());
//			}
//			
//			if (imTurns != null) {
//				System.out.println("Move:" + imTurns.size());
//			}
//		}
//		
//	}
//
//}
