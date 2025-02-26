// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.hal.can.CANJNI;

public class DeviceFinder {

	private ByteBuffer targetID = ByteBuffer.allocateDirect(4);
	private ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);

	private int ctreMask = 0x00040000;

	private HashMap<Integer, String> deviceNames = new HashMap<Integer, String>();
	
	public DeviceFinder() {
	
		deviceNames.put(10, "Pigeon");
		deviceNames.put(11, "LF Drive");
		deviceNames.put(12, "LF Steer");
		deviceNames.put(13, "LF CanCoder");
		deviceNames.put(21, "RF Drive");
		deviceNames.put(22, "RF Steer");
		deviceNames.put(23, "RF CanCoder");
		deviceNames.put(31, "LR Drive");
		deviceNames.put(32, "LR Steer");
		deviceNames.put(33, "LR CanCoder");
		deviceNames.put(41, "RR Drive");
		deviceNames.put(42, "RR Steer");
		deviceNames.put(43, "RR CanCoder");
		
	}
	

	/** helper routine to get last received message for a given ID */
	private long checkMessage(int fullId, int deviceID) {
		try {
			targetID.clear();
			targetID.order(ByteOrder.LITTLE_ENDIAN);
			targetID.asIntBuffer().put(0,fullId|deviceID);

			timeStamp.clear();
			timeStamp.order(ByteOrder.LITTLE_ENDIAN);
			timeStamp.asIntBuffer().put(0,0x00000000);
			
			CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0x1fffffff, timeStamp);
		
			long retval = timeStamp.getInt();
			retval &= 0xFFFFFFFF; /* undo sign-extension */ 
			return retval;
		} catch (Exception e) {
			return -1;
		}
	}
	/** polls for received framing to determine if a device is present.
	 *   This is meant to be used once initially (and not periodically) since 
	 *   this steals cached messages from the robot API.
	 * @return ArrayList of strings holding the names of devices we've found.
	 */
	public ArrayList<String> find() {

		/* get timestamp0 for each device */
		long []ctre_timeStamp0 = new long[63];
		
		
		for(int i=0;i<63;++i) {
			// 0x00040000 is the base ID for CTRE devices
			ctre_timeStamp0[i] = checkMessage(ctreMask, i);
		}

		/* wait 200ms */
		try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		/* get timestamp1 for each device */
		long []ctre_timeStamp1 = new long[63];
		
		for(int i=0;i<63;++i) {
			ctre_timeStamp1[i] = checkMessage(ctreMask, i);
		}

		/* compare, if timestamp0 is good and timestamp1 is good, and they are different, device is healthy */
		

		ArrayList<String> foundDevices = new ArrayList<String>();
		String name = "";

		for(int i=0;i<63;++i) {
			if( ctre_timeStamp0[i]>=0 && ctre_timeStamp1[i]>=0 && ctre_timeStamp0[i]!=ctre_timeStamp1[i]){
				name = "Unknown";
				if (deviceNames.containsKey(i)) {
					name = deviceNames.get(i);
				}
				foundDevices.add("Device: " + name + " ID: " + i);
			}
		}
		return foundDevices;
	}
}


