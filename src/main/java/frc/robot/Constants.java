/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.utils.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Operator Interface 
    public static final int DRIVER_REMOTE_PORT = 0;
    public static final int DRIVER_RIGHT_AXIS = 4;
    public static final int DRIVER_LEFT_AXIS = 1;
    public static final int PARTNER_REMOTE_PORT = 1;

    // DriveTrain Constants
    public static final int LEFT_TALON_LEADER    = 4;
    public static final int LEFT_TALON_FOLLOWER  = 3;
    public static final int RIGHT_TALON_LEADER   = 5;
    public static final int RIGHT_TALON_FOLLOWER = 6;

    public final static int kSensorUnitsPerRotation = 512;
	public final static double kNeutralDeadband = 0.001;
	
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop
	 * 	                                    			  kP   kI    kD     kF             Iz    PeakOut */
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/4000.0,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
	
	public final static int kPrimaryPIDSlot = 0; // any slot [0,3]
}