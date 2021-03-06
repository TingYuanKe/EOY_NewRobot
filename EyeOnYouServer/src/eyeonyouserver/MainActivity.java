package eyeonyouserver;
import java.io.File;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Enumeration;


public class MainActivity {
	public static MainServerSocket servsock = null;
	public static Socket sock = null;
	
	public final static int SOCKET_PORT = 8888; //
	public final static int BACK_LOG = 10;
	public static InetAddress addr;
	// public final static String FILE_TO_SEND =
	// "C:/Users/TingYuanKeke/Desktop/test/test01.txt"; // you may change this
	public final static int FILE_SIZE = 1024 * 1024;
	// FIle recieve folder
	
	public static void main(String[] args) throws IOException{
		System.out.println("==========Java Server Init============");
		
		//clean sensing files in the IMUData and SkeletonData
		for(File file:new java.io.File("/home/newrobot/data/SkeletonData").listFiles()) {
			 if (!file.isDirectory()) 
			        file.delete();
		}
		
		for(File file:new java.io.File("/home/newrobot/data/IMUData").listFiles()) {
			 if (!file.isDirectory()) 
			        file.delete();
		}
		   
		//find current network Interfaces 
		Enumeration<NetworkInterface> n = NetworkInterface.getNetworkInterfaces();
		NetworkInterface e = n.nextElement();
	    System.out.println("Interface: " + e.getName());
	    
	    //get the IP address of certain network interface
		NetworkInterface nif = NetworkInterface.getByName("wlp2s0");
		Enumeration<InetAddress> nifAddresses = nif.getInetAddresses();
		for (; nifAddresses.hasMoreElements();) {
            addr = nifAddresses.nextElement();
            System.out.println("Current IP Address " + addr.getHostAddress());
		}
		nifAddresses = nif.getInetAddresses();
		System.out.println("Interface: " + addr);
		//serverSocket.bind(new InetSocketAddress(nifAddresses.nextElement(),8888));
		try{
			servsock = new MainServerSocket(addr ,BACK_LOG , SOCKET_PORT);
			
			if (servsock.isConnected()) {
				servsock.run();
			}
		}
		catch (Exception errs) {
			System.err.println(errs.getMessage());
			System.err.println("Java server has been excuted,\n please stop all the server task and run again!");
		}
		finally{
			// terminate server socket
			if (servsock != null)
				servsock.close();
			
			System.exit(1);
		}
	}
}