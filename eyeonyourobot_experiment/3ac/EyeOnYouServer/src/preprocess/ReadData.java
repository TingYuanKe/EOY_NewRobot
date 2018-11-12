package preprocess;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import com.opencsv.CSVReader;

import data.HeadPos;
import data.Inertial;
import data.Skeleton;

/**
* This class is used to read skeleton and inertial data from files.
*
* @author  WeiChun
*/
public class ReadData {
	
	/**
	* read skeleton data and store in Skeleton objects 
	*   
	* @param: file name for skeleton data
	* 
	* @return: list of Skeleton objects 
	*/
	public static ArrayList<Skeleton> readKinect(String file) {
		
		ArrayList<Skeleton> jointpos = new ArrayList<Skeleton>();
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				Skeleton aJointPos = new Skeleton(Double.valueOf(line[0]), Double.valueOf(line[1]), Double.valueOf(line[2]));
				jointpos.add(aJointPos);
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return jointpos;
	
	}
	
	/**
	* read skeleton data and smooth in the way of Moving Average
	* then store smoothed data in Skeleton objects 
	*   
	* @param: file name for skeleton data
	* 
	* @return: list of Skeleton objects 
	*/
	public static ArrayList<Skeleton> readKinect_smooth(String file) {		// 5 span moving average
		
		ArrayList<Skeleton> jointpos = new ArrayList<Skeleton>();
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				Skeleton aJointPos = new Skeleton(Double.valueOf(line[0]), Double.valueOf(line[1]), Double.valueOf(line[2]));
				jointpos.add(aJointPos);
			}
			for (int i = 0; i < jointpos.size(); i++) {
				if (i > 1 && i < jointpos.size()-2) {
					double[] kinectpoint = new double[3];
					for (int j = 0; j < 3; j++) {
						kinectpoint[j] = (jointpos.get(i-2).getRight_wrist()[j] + jointpos.get(i-1).getRight_wrist()[j] + jointpos.get(i).getRight_wrist()[j] + 
								jointpos.get(i+1).getRight_wrist()[j] + jointpos.get(i+2).getRight_wrist()[j])/5;
					}
					jointpos.get(i).setRight_wrist(kinectpoint);
				} else if (i == 1 || i == jointpos.size()-2) {
					double[] kinectpoint = new double[3];
					for (int j = 0; j < 3; j++) {
						kinectpoint[j] = (jointpos.get(i-1).getRight_wrist()[j] + jointpos.get(i).getRight_wrist()[j] + jointpos.get(i+1).getRight_wrist()[j])/3;
					}
					jointpos.get(i).setRight_wrist(kinectpoint);
				} else {
					continue;
				}
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return jointpos;
	
	}
	
	/**
	* read inertial data and store in Inertia objects 
	*   
	* @param: file name for inertial data
	* 
	* @return: list of Inertia objects 
	*/
	public static ArrayList<Inertial> readIMU(String file) {
		
		ArrayList<Inertial> imu_6 = new ArrayList<Inertial>();
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				Inertial sixaxis = new Inertial(Double.valueOf(line[0]), Double.valueOf(line[1]), Double.valueOf(line[2]));
				imu_6.add(sixaxis);
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return imu_6;
	
	}
	
	/**
	* read XY positions of head in the video frames and store in HeadPos objects 
	*   
	* @param: file name for skeleton data
	* 
	* @return: list of HeadPos objects 
	*/
	public static ArrayList<HeadPos> readHead(String file) {
		
		ArrayList<HeadPos> headXY = new ArrayList<HeadPos>();
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				HeadPos aHeadXY = new HeadPos(Double.valueOf(line[19]), Double.valueOf(line[20]));
				headXY.add(aHeadXY);
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return headXY;
	}
	
	public static int readID(String file) {
		
		int userID = -1;
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			if ((line = cr.readNext()) != null) {
				userID = Integer.valueOf(line[18]);
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return userID;
	}
	
}
