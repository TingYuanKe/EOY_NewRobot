package eyeonyouserver;
import java.io.IOException;

import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.Timer;
import java.util.TimerTask;

import eyeonyouserver.SocketClientWithSensing;
import pairing.PID;

public class MainServerSocket {
	private int port;
	private ServerSocket servsock;
	private boolean isConnected = false;
	
	public static SocketClientWithSensing clientRunPID = null;
    static String ipaddress = "localhost";
    
	private Timer timerStartPairing = new Timer();
	private Timer timerEndPairing = new Timer();
	public static boolean isPairing = false;
	public int collectInterval = 4000;
	public int pairingInterval =1000;
	
	public MainServerSocket(int port) throws IOException {
		this.port = port;
		this.servsock = new ServerSocket(port);
		isConnected=true;
		System.out.println("EyeOnYouServer starts!");
	}

	public void run() {
		clientRunPID = new SocketClientWithSensing(ipaddress);
		
		timerStartPairing.scheduleAtFixedRate(new TimerTask() {
			 @Override
			 public void run() {
				 isPairing = true;
				 clientRunPID.requestKinectKeepSkeleton();
				 }
			 }, 500, collectInterval);
		 timerEndPairing.scheduleAtFixedRate(new TimerTask() {
			 @Override
			 public void run() {
				 isPairing = false;
				 }
			 }, 500+collectInterval+pairingInterval, collectInterval);
		 
		ExecutorService executor = Executors.newFixedThreadPool(5);
		while (true)// ¥Ã»·°õ¦æ
		{
			Socket sock = null;
			try {
				System.out.println("waiting client...");
				sock = servsock.accept();
				System.out.println("Accepted connection : " + sock);
			} catch (java.io.IOException e) {
				e.printStackTrace();
			}
			
			 Runnable worker = new FileRecieveThread(sock);
			 executor.execute(worker);
		}
	}
	
	public void close() throws IOException{
		this.servsock.close();
	}
	public boolean isConnected() {
		return this.isConnected;
	}
}
