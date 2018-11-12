package preprocess;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import com.opencsv.CSVReader;
import com.opencsv.CSVWriter;

/**
* This class is used to process data of skeleton files. 
*
* @author  WeiChun
*/
public class BodyExtraction {
	
	/**
	* record all body IDs 
	*   
	* @param: .csv file which record joins position of users
	* 
	* @return: list of IDs
	*/
	public static ArrayList<Integer> bodyCount(String file) {
		ArrayList<Integer> ids = new ArrayList<Integer>();
		CSVReader cr = null;
		
		try {
			cr = new CSVReader(new FileReader(file));
			String[] line;
			while ((line = cr.readNext()) != null) {
				if(ids.isEmpty() || !ids.contains(Integer.valueOf(line[6]))) {		// ID is in the 18th column
					ids.add(Integer.valueOf(line[6]));
				}
			}
			cr.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		return ids;
		
	}
	
	/**
	* write joins position to separate file
	*   
	* @param: .csv file which is used to record joints position of all users
	* @param: .csv file which is used to record joints position of each user
	* @param: skeleton id
	*/
	public static void bodyWriter(String inputfile, String outputfile, int id) {
		CSVReader cr = null;
		
		try {
			cr = new CSVReader(new FileReader(inputfile));
			CSVWriter cw = new CSVWriter(new FileWriter(outputfile), ',', CSVWriter.NO_QUOTE_CHARACTER);
			
			String[] line;
			
			while((line = cr.readNext()) != null) {
				if (Integer.valueOf(line[6]) == id) {			// ID is in the 18th column
					cw.writeNext(line);
				}
        	}
			cw.close();
			cr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
	}
	
	public static void main(String[] args) {
		String rootDir = "C:/Users/Public/Data";
		String[] files = {"set1", "set2", "set3"};
		ArrayList<Integer> ids;
		
		for (int i = 0; i < 3; i++) {
			ids = bodyCount(rootDir + "/KINECTData/Evaluation/2people/" + files[i] + ".csv");
			System.out.println(ids.size());
			
			/*for (int j = 0; j < ids.size(); j++) {
				bodyWriter(rootDir + "/KINECTData/Evaluation/" + files[i] + ".csv", 
						rootDir + "/KINECTData/Evaluation/" + files[i] + "_" + ids.get(j) + ".csv", ids.get(j));
			}*/
		}
	}

}
