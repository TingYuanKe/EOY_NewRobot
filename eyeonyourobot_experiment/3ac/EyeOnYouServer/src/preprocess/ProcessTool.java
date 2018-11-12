package preprocess;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;

/**
* This class have some functions for data processing
*
* @author  WeiChun
*/
public class ProcessTool {
	
	/**
	* transform .txt to .csv 
	*   
	* @param: .txt file name
	* @param: .csv file name
	* 
	* @return: length of raw file 
	*/
	public static int reformat(String inputfile, String outputfile) {
		int len = 0;
		try {
			BufferedReader br = new BufferedReader(new FileReader(inputfile));
			CSVWriter cw = new CSVWriter(new FileWriter(outputfile), ',', CSVWriter.NO_QUOTE_CHARACTER);
			
			String line;
			String[] raw = new String[4];	// 3 acceleration force and 1 time stamp
			
			while((line = br.readLine()) != null) {
        		raw = line.split(",");
        		cw.writeNext(raw);
        		len++;
        	}
			cw.close();
			br.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return len;
    }
	
	/**
	* list files in a directory
	*   
	* @param: file object
	* 
	* @return: a file name list 
	*/
	public static ArrayList<String> listFilesForFolder(final File folder) {
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
	* segment data into certain interval
	*   
	* @param: original .csv file name
	* @param: .csv file name
	* @param: offset
	* @param: length
	* 
	* @return: length of raw file 
	*/
	public static int ComputationSegment(String inputfile, String outputfile, int offset, int length) {
		int len = 0;
		try {
			CSVReader cr = new CSVReader(new FileReader(inputfile));
			CSVWriter cw = new CSVWriter(new FileWriter(outputfile), ',', CSVWriter.NO_QUOTE_CHARACTER);
			
			String line[];
			
			while((line = cr.readNext()) != null) {
				if (len >= offset && len < (offset + length)) {
        			cw.writeNext(line);
				}
        		len++;
        	}
			cw.close();
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return len;
    }
	
	/**
	* synchronize the starter of skeleton data and inertial data
	*   
	* @param: skeleton file
	* @param: inertia file
	* 
	* @return: offset of inertia data 
	*/
	public static int getStarter(String skeletonFile, String inertiaFile) {
		CSVReader cr_s = null, cr_i = null;
		SimpleDateFormat parser = new SimpleDateFormat("HH:mm:ss:SSS");
		int offset = 0;
		Date VideoTime = null;
		Date InertiaTime = null;
		Date InertiaTime_next = null;
		
		try {
			cr_s = new CSVReader(new FileReader(skeletonFile));
			cr_i = new CSVReader(new FileReader(inertiaFile));
			
			String[] line;
			if ((line = cr_s.readNext()) != null) {
				VideoTime = parser.parse(line[21]);
			}
			cr_s.close();
			//System.out.println(VideoTime.getTime());
			
			if ((line = cr_i.readNext()) != null) {
				InertiaTime = parser.parse(line[3]);
			}
			while ((line = cr_i.readNext()) != null) {
				InertiaTime_next = parser.parse(line[3]);
				if (VideoTime.before(InertiaTime) || (VideoTime.after(InertiaTime) && VideoTime.before(InertiaTime_next)) || VideoTime.equals(InertiaTime)) {
					break;
				}
				offset++;
				InertiaTime = InertiaTime_next;
			}
			//System.out.println(InertiaTime.getTime());
			//System.out.println(InertiaTime_next.getTime());
			cr_i.close();
		} catch (ParseException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return offset;
	}

	public static void main(String[] args) {
		String rootDir_k = "J:/Users/Wei/Desktop/Data/KINECTData/Evaluation/InHand/";
		String rootDir_i = "J:/Users/Wei/Desktop/Data/IMUData/Evaluation/InHand/";
		
		String fileName = "inhand5";
		
		/* File Segmentation for skeleton data */
		for (int i = 0; i < 6; i++) {
			ComputationSegment(rootDir_k + fileName + ".csv", rootDir_k + fileName + "_" + (i+1) + ".csv", i * 300, 300);
		}
		
		reformat(rootDir_i + fileName + ".txt", rootDir_i + fileName + ".csv");
		/* File Segmentation for inertial data */
		for (int i = 0; i < 6; i++) {
			ComputationSegment(rootDir_i + fileName + ".csv", rootDir_i + fileName + "_" + (i+1) + ".csv", i * 1000, 1000);
		}
		
	}

}
