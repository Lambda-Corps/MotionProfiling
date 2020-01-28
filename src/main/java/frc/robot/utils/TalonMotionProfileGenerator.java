/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

/**
 * Add your docs here.
 */
public class TalonMotionProfileGenerator {

    public static BufferedTrajectoryPointStream generateTalonProfile( String path, String file, double metersPerRevolution, int ticksPerRotation, boolean reverseDirection){
        
        List<double[]> profile = genArrayFromXeroFile(path + file);
        
        BufferedTrajectoryPointStream bufferedStream = new BufferedTrajectoryPointStream();
        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                    // automatically, you can alloc just one
        /* Insert every point into buffer, no limit on size */
        int totalCnt = profile.size();
        int direction = reverseDirection ? -1 : 1;

        for( int i = 0; i < totalCnt; i++){
            double [] entry = profile.get(0);
            /* for each point, fill our structure and pass it to API */
            point.timeDur = (int)entry[2] * 1000;
            point.position = direction * (entry[0] * (1 / metersPerRevolution)) * ticksPerRotation; // Convert Revolutions to Units
            point.velocity = direction * (entry[1] * (1 / metersPerRevolution)) * ticksPerRotation / 600.0; // Convert RPM to Units/100ms
            point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            point.zeroPos = (i == 0); /* set this to true on the first point */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            bufferedStream.Write(point);
        }

        return bufferedStream;
    }

    /**
     * Class to generate a multi-dimensional array of doubles representing POSITION, VELOCITY, TIME.
     * 
     * @param String Path to CSV file
     */
    private static List<double[]> genArrayFromXeroFile(String path){
        
        // Xero format is: "time","x","y","position","velocity","acceleration","jerk","heading"
        File file= new File(path);

        // this gives you a 2-dimensional array of strings
        List<double[]> lines = new ArrayList<double[]>();
        Scanner inputStream;

        try{
            inputStream = new Scanner(file);

            while(inputStream.hasNext()){
                String line= inputStream.next();
                if( line.charAt(0) == '"'){
                    continue;
                }

                String[] values = line.split(",");

                // to put the timestampp into milliseconds for the Talon
                double [] pvt = {Double.parseDouble(values[3]),Double.parseDouble(values[4]),Double.parseDouble(values[0])};
                // this adds the currently parsed line to the 2-dimensional string array
                lines.add(pvt);
            }

            inputStream.close();
        }catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        return lines;
    }
}
