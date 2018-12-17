package eyeonyouserver;
import java.io.IOException;


import java.net.ServerSocket;
import java.net.Socket;
import java.net.InetAddress;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.Timer;
import java.util.TimerTask;


import eyeonyouserver.SocketClientWithDepthCam;
import pairing.PID;

public class MainServerSocket {
	private int port;
	private ServerSocket servsock;
	private boolean isConnected = false;
	
	public static SocketClientWithDepthCam clientRunPID = null;
    static String ipaddress = "localhost";
    
	private Timer timerStartPairing = new Timer();
	private Timer timerEndPairing = new Timer();
	public static boolean isPairing = false;
	public int collectInterval = 4000;
	public int pairingInterval =1000;
	
	public MainServerSocket(InetAddress ip, int back_log, int port) throws IOException {
		
		
		this.port = port;
		this.servsock = new ServerSocket(port,back_log,ip);

		isConnected = true;
		System.out.println("\n==========Java Server Start============");
	}

	public void run() {
		
		clientRunPID = new SocketClientWithDepthCam(ipaddress);
		
		if (clientRunPID.isConnected()) {
			
			// Server sensing stop and run PID time ack
			timerStartPairing.scheduleAtFixedRate(new TimerTask() {
				@Override
				public void run() {
					isPairing = true;
					// System.out.println("Inertial ack true");
					clientRunPID.requestKinectKeepSkeleton();
				 	}
			 	}, 500, collectInterval);
			
			// Server keep sensing time ack
			timerEndPairing.scheduleAtFixedRate(new TimerTask() {
				@Override
				public void run() {
					isPairing = false;
					// System.out.println("Inertial ack false");
				 	}
			 	}, 500+collectInterval+pairingInterval, collectInterval);
		 
			ExecutorService executor = Executors.newFixedThreadPool(5);
			while (true)// �û�����
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
	}
	
	public void close() throws IOException{
		this.servsock.close();
	}
	public boolean isConnected() {
		return this.isConnected;
	}
}
