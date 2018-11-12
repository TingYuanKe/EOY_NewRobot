package eyeonyouserver;


import java.io.*;
import java.net.*;

import pairing.PID;
/**
 * @author Richard Yi-Chia TSAI, TingYuan
 */



public class SocketClientWithSensing {
    private Listener listener = null;
    private Sender sender = null;
    public boolean connected = false;
    
    
    class StartPairingThread extends Thread {

        public void run() {
        	System.out.println("\n==========\n3. The EyeOnYouServer is performing PID.\n==========\n");
        	PID.startPairing();
        }


    }
    
    // Listener
    class Listener extends Thread {
        Socket conn = null;
        boolean listening = true;

        public Listener(Socket conn) {
            this.conn = conn;
            this.setName("JavaClientSocketListener");
            this.start();
        }

        @Override public void run() {

            InputStream instream = null;

            try {
                BufferedReader reader =
                    new BufferedReader(new InputStreamReader(conn.getInputStream()));
                	
                while ( listening ) {
//                	System.out.println("Keep listening c++ sensing");
                    String xml = reader.readLine();
                    if ( xml == null ) {
                        // Connection lost
                        return;
                    }
                    
                    System.out.println("XML sent from C++: " + xml + "\n\n");
                    // Listen runPID command sent from Sensing program because it finished preparing the VSFile.csv
                    if(xml.equals("runPID")) {
                    	(new Thread(new StartPairingThread())).start();;
                    }
                }
            }
            catch ( StreamCorruptedException sce) {
                // skip over the bad bytes
                try {
                    if ( instream != null )
                        instream.skip(instream.available());
                }
                catch ( Exception e1 ) {
                    listening = false;
                }
            }
            catch ( Exception e ) {
                e.printStackTrace();
                listening = false;
            }
        }
    }

    // Sender
    class Sender {
        static final String HOSTNAME = "<Request><Name>GetHostname</Name></Request>";
        static final String MEMORY = "<Request><Name>GetMemory</Name></Request>";
        static final String RANDOM_NUM = "<Request><Name>GetRandomNumber</Name></Request>";
        static final String GET_SKELETON = "<Request><Name>GetKinectKeepSkeleton</Name></Request>";
        static final String TAG_PROFILE = "<Request><Name>GetKinectTagProfile</Name></Request>";
        
        Socket conn;
        BufferedOutputStream os = null;

        public Sender(Socket conn) {
            try {
                this.conn = conn;
                this.conn.setTcpNoDelay(true);
                this.os = new BufferedOutputStream( conn.getOutputStream() );
            }
            catch ( Exception e ) {
                e.printStackTrace();
            }
        }

        public void requestHostname() {
            serializeAndSendMessage(HOSTNAME);
        }

        public void requestMemory() {
            serializeAndSendMessage(MEMORY);
        }

        public void requestRandomNumber() {
            serializeAndSendMessage(RANDOM_NUM);
        }
        
        public void requestKinectKeepSkeleton() {
        	System.out.println("\n==========\n1. The EyeOnYouSensing is collecting skeleton data by Kinect.\n==========\n");
            serializeAndSendMessage(GET_SKELETON);
        }
        public void requestKinectTagProfile() {
        	System.out.println("\n==========\n1. The EyeOnYouSensing is tagging name (PID result) on respective skeletons.\n==========\n");
            serializeAndSendMessage(TAG_PROFILE);
        }

        private void serializeAndSendMessage(String msg) {
            try {
                os.write( msg.getBytes() );
                os.flush();
            }
            catch ( Exception e ) {
                e.printStackTrace();
            }
        }
    }

    // SocketClientRunPID Constructor
    public SocketClientWithSensing(String IPAddress) {
        try {
            // Connect to the server at the given address on port 8081
            if ( IPAddress == null || IPAddress.length() == 0 )
                IPAddress = "localhost";
            Socket conn = new Socket( IPAddress, 8081 );
            conn.setTcpNoDelay(true);
            this.listener = new Listener(conn);
            this.sender = new Sender(conn);
            this.connected = true;
        }
        catch ( Exception e ) {
            connected = false;
            e.printStackTrace();
        }
    }

    public boolean isConnected() {
        return connected;
    }

    public void requestHostname() {
        sender.requestHostname();
    }

    public void requestMemory() {
        sender.requestMemory();
    }

    public void requestRandomNumber() {
        sender.requestRandomNumber();
    }
    
    public void requestKinectKeepSkeleton() {
        sender.requestKinectKeepSkeleton();
    }

    public void requestKinectTagProfile() {
        sender.requestKinectTagProfile();
    }
}

