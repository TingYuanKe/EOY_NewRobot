package eyeonyouserver;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

public class FileRecieveThread extends Thread {
	String rootDir = "/home/newrobot/data";
	protected final static int FILE_SIZE = 1024 * 1024;
	byte[] mybytearray = new byte[FILE_SIZE];
	protected String whoSent;
	protected String FILE_TO_RECEIVED;
	protected String FILE_TO_PAIRING;
	protected String FILE_TO_PAIRING_CVS;
	
	protected boolean fileCleaned= true;

	protected Socket socket;
	protected FileOutputStream fos = null;
	protected BufferedOutputStream bos = null;
	protected InputStream is = null;
	protected DataInputStream dis = null;

	public FileRecieveThread(Socket clientSocket) {

		this.socket = clientSocket;
		try {
			is = this.socket.getInputStream();
			dis = new DataInputStream(is);

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	// keep recieve data line from socket outputStream
	@Override
	public void run() {
		try {
			whoSent = dis.readLine();
			FILE_TO_RECEIVED = rootDir + "/IMUData/" + whoSent + "_buffer.txt";
			FILE_TO_PAIRING = rootDir + "/IMUData/" + whoSent + ".txt";
			FILE_TO_PAIRING_CVS =rootDir + "/IMUData/" + whoSent + ".csv";
			fos = new FileOutputStream(FILE_TO_RECEIVED);
			bos = new BufferedOutputStream(fos);

			// Receive file
			System.out.println(whoSent);
			do {
				int lenBuffer = 0;
				while ((lenBuffer = dis.read(mybytearray)) > 0) {
					bos.write(mybytearray, 0, lenBuffer);
					0
					if(MainServerSocket.isPairing == true) {
						System.out.println("\n==========\n2. The EyeOnYouServer is collecting inertial data from UEs.\n==========\n");
						while(MainServerSocket.isPairing == true) {
							if(fileCleaned == false){
								bos.close();
								fos.close();
								
								File f1 = new File(FILE_TO_RECEIVED);
							    File f2 = new File(FILE_TO_PAIRING);
							    InputStream in = new FileInputStream(f1);
							    OutputStream out = new FileOutputStream(f2);
							    
							    byte[] buf = new byte[1024];
							    int lenPairing;
							    
							    while ((lenPairing = in.read(buf)) > 0){
							      out.write(buf, 0, lenPairing);
							    }
							    in.close();
					            out.close();
						      
						      	fos = new FileOutputStream(FILE_TO_RECEIVED);
						      	bos = new BufferedOutputStream(fos);
						
						        fileCleaned=true;
							}
						}
					}
					fileCleaned = false;
					
					System.out.println(Thread.currentThread().getName() + " -> " + whoSent + " is sending data.");
				}
			} while (dis.readLine() != null);

			System.out.println(whoSent + " just turn off the sending process.");
			boolean txtDelete = new File(FILE_TO_PAIRING).delete();
			boolean csvDelete = new File (FILE_TO_PAIRING_CVS).delete();
			System.out.println(whoSent+".txt file has been deleted");
			
			fos.close();
			dis.close();

		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void close() throws IOException {

		bos.close();
		fos.close();
		dis.close();

	}
}
