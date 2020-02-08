/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class InstrumentProfile {
    static int _loops = 0;

	static boolean _bPrintValues = false;

	public static void printLine(String s) {
		System.out.println(s);
	}

	public static void loop(boolean bPrintValues, TalonSRX left, TalonSRX right) {
		if (!_bPrintValues && bPrintValues) {
			/* user just pressed button, immediete print */
			_loops = 999;
		}
		/* if button is off, don't print */
		if (bPrintValues == false) {
			/* reset so we don't print */
			_loops = 0;
		}
		/* save for next compare */
		_bPrintValues = bPrintValues;

		/* build string and print if button is down */
		if (++_loops >= 10) {
			_loops = 0;
			/* get status info */
            MotionProfileStatus leftStatus = new MotionProfileStatus();
            MotionProfileStatus rightStatus = new MotionProfileStatus();
			left.getMotionProfileStatus(leftStatus);

			String line = "";
			line += " L  topBufferRem: " + leftStatus.topBufferRem + " \tR  topBufferRem: " + rightStatus.topBufferRem + "\n";
			line += " L  topBufferCnt: " + leftStatus.topBufferCnt + " \tR  topBufferRem: " + rightStatus.topBufferCnt + "\n";
			line += " L  btmBufferCnt: " + leftStatus.btmBufferCnt + " \tR  topBufferRem: " + rightStatus.btmBufferCnt + "\n";
			line += " L  hasUnderrun: " + leftStatus.hasUnderrun + " \tR  topBufferRem: " + rightStatus.hasUnderrun + "\n";
			line += " L  isUnderrun: " + leftStatus.isUnderrun + " \tR  topBufferRem: " + rightStatus.isUnderrun + "\n";
			line += " L  activePointValid: " + leftStatus.activePointValid + " \tR  topBufferRem: " + rightStatus.activePointValid + "\n";
			line += " L  isLast: " + leftStatus.isLast + " \tR  topBufferRem: " + rightStatus.isLast + "\n";
			line += " L  profileSlotSelect0: " + leftStatus.profileSlotSelect + " \tR  topBufferRem: " + rightStatus.profileSlotSelect +  "\n";
			line += " L  profileSlotSelect1: " + leftStatus.profileSlotSelect1 + " \tR  topBufferRem: " + rightStatus.profileSlotSelect1 + "\n";
			line += " L  outputEnable: " + leftStatus.outputEnable.toString() + " \tR  topBufferRem: " + rightStatus.outputEnable.toString() + "\n";
			line += " L  : " + leftStatus.timeDurMs + " \tR  topBufferRem: " + rightStatus.timeDurMs + "\n";

			printLine(line);
		}
	}
}
