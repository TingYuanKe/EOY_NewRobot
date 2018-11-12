/*
 * roombacomm.EyeOnYouRobotTracking -- test out the Drive command from Kinect Sensing
 *
 *  Copyright (c) 2018 Richard Yi-Chia TSAI richardtsai.cs05g@nctu.edu.tw
 *  
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with this library; if not, write to the
 *  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 */

package roombacomm.eyeonyourobot;

import javax.swing.*;

import roombacomm.eyeonyourobot.SocketClient;
import roombacomm.RoombaCommSerial;

import java.awt.*;
import java.awt.event.*;

import java.util.Timer;
import java.util.TimerTask;

/**
   Drive the Roomba with the arrow keys in real-time
  <p>
   Run it with something like: <pre>
    java roombacomm.DriveRealTime /dev/cu.KeySerial1<br>
   Usage: 
   roombacomm.DriveRealTime serialportname [protocol] [options]<br>
   where: 
   protocol (optional) is SCI or OI 
   [options] can be one or more of:
   -debug       -- turn on debug output
   -hwhandshake -- use hardware-handshaking, for Windows Bluetooth
   -nohwhandshake -- don't use hardware-handshaking
   </pre>
*/ 
public class DriveBasedOnKinect {

	static Timer timer = new Timer();
	
	static SocketClient client = null;
    static String ipaddress = "localhost";
    
    static String usage = 
        "Usage: \n"+
        "  roombacomm.DriveRealTime <serialportname> [protocol] [options] \n" +
        "where: protocol (optional) is SCI or OI\n" +
        "[options] can be one or more of:\n"+
        "-debug       -- turn on debug output\n"+
        "-hwhandshake -- use hardware-handshaking, for Windows Bluetooth\n"+
        "-nohwhandshake -- don't use hardware-handshaking\n"+
        "\n";
    static boolean debug = false;
    static boolean hwhandshake = false;
    static String drivetowhere = "";
    static int driveunit = 0;
    
    static RoombaCommSerial roombacomm;
    JTextArea displayText;

    public static void main(String[] args) {
    	
    	// Attempt to connect to server
        client = new SocketClient(ipaddress);
        roombacomm = new RoombaCommSerial();
        roombacomm.debug = debug;
        
        if( ! roombacomm.connect( "COM3" ) ) {
            System.out.println("Couldn't connect to COM3");
            System.exit(1);
        }
        roombacomm.setProtocol("OI");
        
        timer.scheduleAtFixedRate(new TimerTask() {
        	  @Override
        	  public void run() {
        	    // Your database code here
        		  client.requestRandomNumber();
        	  }
        	}, 1000, 1000);
        
        if( args.length < 1 ) {
            System.out.println( usage );
            System.exit(0);
        }
    }
    
    /***
     * 
     * @param rec_drivetowhere
     * @param rec_driveunit
     */
    public static void DriveAction(String rec_drivetowhere, int rec_driveunit) {
    	drivetowhere = rec_drivetowhere;
    	driveunit = rec_driveunit;
        
        System.out.println("Roomba startup");
        roombacomm.startup();
        roombacomm.control();
        roombacomm.pause(50);
  
        System.out.println(rec_drivetowhere);
        System.out.println(rec_driveunit);
        
        if(drivetowhere.equals("stop"))
        	roombacomm.stop();
        else if(drivetowhere.equals("forward"))
        {
        	roombacomm.setSpeed(80);
        	roombacomm.goForward(driveunit);
        }
        else if(drivetowhere.equals("backward"))
        {
        	roombacomm.setSpeed(80);
        	roombacomm.goBackward(driveunit);
        }
        else if(drivetowhere.equals("spinleft"))
        	roombacomm.spinLeft(driveunit);
        else if(drivetowhere.equals("spinright"))
        	roombacomm.spinRight(driveunit);
    }
}