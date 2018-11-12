package preprocess;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
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
				Skeleton aJointPos = new Skeleton(Double.valueOf(line[0]), Double.valueOf(line[1]), Double.valueOf(line[2]), Double.valueOf(line[3]), Double.valueOf(line[4]), Double.valueOf(line[5]));
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
				Skeleton aJointPos = new Skeleton(Double.valueOf(line[0]), Double.valueOf(line[1]), Double.valueOf(line[2]), Double.valueOf(line[3]), Double.valueOf(line[4]), Double.valueOf(line[5]));
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
	public static ArrayList<Inertial> readIMU(String file, double thresholdInertialAcc) {
		
		ArrayList<Inertial> imu_6 = new ArrayList<Inertial>();
		double[] acc = new double[3];
		double[] gyro = new double[3];
		
		CSVReader cr = null;
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				if(Math.abs(Double.valueOf(line[0])) > thresholdInertialAcc)
					acc[0] = Double.valueOf(line[0]);
				else
					acc[0] = 0;
				if(Math.abs(Double.valueOf(line[1])) > thresholdInertialAcc)
					acc[1] = Double.valueOf(line[1]);
				else
					acc[1] = 0;
				if(Math.abs(Double.valueOf(line[2])) > thresholdInertialAcc)
					acc[2] = Double.valueOf(line[2]);
				else
					acc[2] = 0;
				if(Math.abs(Double.valueOf(line[3])) > thresholdInertialAcc)
					gyro[0] = Double.valueOf(line[3]);
				else
					gyro[0] = 0;
				if(Math.abs(Double.valueOf(line[4])) > thresholdInertialAcc)
					gyro[1] = Double.valueOf(line[4]);
				else
					gyro[1] = 0;
				if(Math.abs(Double.valueOf(line[5])) > thresholdInertialAcc)
					gyro[2] = Double.valueOf(line[5]);
				else
					gyro[2] = 0;
				
				Inertial sixaxis = new Inertial(acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
				imu_6.add(sixaxis);
			}
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return imu_6;
	}
	
	public static int countFileLine(String filename) throws IOException {
	    InputStream is = new BufferedInputStream(new FileInputStream(filename));
	    try {
	    byte[] c = new byte[1024];
	    int count = 0;
	    int readChars = 0;
	    boolean empty = true;
	    while ((readChars = is.read(c)) != -1) {
	        empty = false;
	        for (int i = 0; i < readChars; ++i) {
	            if (c[i] == '\n') {
	                ++count;
	            }
	        }
	    }
	    return (count == 0 && !empty) ? 1 : count;
	    } finally {
	    is.close();
	   }
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
