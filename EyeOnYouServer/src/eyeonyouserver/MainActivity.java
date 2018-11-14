package eyeonyouserver;
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
	// public final static String FILE_TO_SEND =
	// "C:/Users/TingYuanKeke/Desktop/test/test01.txt"; // you may change this
	public final static int FILE_SIZE = 1024 * 1024;
	// FIle recieve folder
	
	public static void main(String[] args) throws IOException{
		
		Enumeration<NetworkInterface> n = NetworkInterface.getNetworkInterfaces();
		for (; n.hasMoreElements();)
        {
                NetworkInterface e = n.nextElement();
                System.out.println("Interface: " + e.getName());
                Enumeration<InetAddress> a = e.getInetAddresses();
                for (; a.hasMoreElements();)
                {
                        InetAddress addr = a.nextElement();
                        System.out.println("  " + addr.getHostAddress());
                }
        }
		
		try{
			//showing IP
			servsock = new MainServerSocket(SOCKET_PORT);
			InetAddress ip= InetAddress.getLocalHost();
			System.out.println("Current IP Address : " + ip.getHostAddress());
			
			if(servsock.isConnected()){
				servsock.run();
			}
		} finally{
			if (servsock != null)
				servsock.close();
		}
	}
}