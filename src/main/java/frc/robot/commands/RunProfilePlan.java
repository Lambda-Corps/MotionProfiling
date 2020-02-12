/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.TalonMotionProfileGenerator;

public class RunProfilePlan extends CommandBase {
  private final DriveTrain m_dt;
  private final XboxController m_driver_controller;
  private final SendableChooser<String> m_chooser;
  private String PROFILE_PATH_DIR;
  private String m_path_str = "";
  private boolean m_isDone = false;
  private BufferedTrajectoryPointStream m_leftTraj, m_rightTraj;
  private NetworkTableEntry m_status;
  /**
   * Creates a new RunProfilePlan.
   */
  public RunProfilePlan(DriveTrain dt, SendableChooser<String> chooser, XboxController cont) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    PROFILE_PATH_DIR = Filesystem.getDeployDirectory().getPath();
    // We're going to append the file name from the chooser, so make sure the 
    // path to the file is well formed with a trailing /
    if(! PROFILE_PATH_DIR.endsWith("/") ){
      PROFILE_PATH_DIR += "/";
    }
    m_driver_controller = cont;
    m_dt = dt;
    m_chooser = chooser;
    addRequirements(m_dt);

    m_status = Shuffleboard.getTab("ProfileTest").add("Run Profile Status", "Not Run Yet").withWidget(BuiltInWidgets.kTextView).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 1 - Grab the left and right trajectories for a given profile
    // 2 - Turnthose  trajectories into BufferedTrajectory objects for the Talon
    // 3 - Reset the current Motion Profile status of the talons
    // 4 - Reset the boolean (done) condition
    // 5 - Start the profile from initialize
    m_status.forceSetString("Initialize");
    m_path_str = PROFILE_PATH_DIR + m_chooser.getSelected();

    m_leftTraj = TalonMotionProfileGenerator.generateTalonProfile(m_path_str, "left.csv", m_dt.getMetersPerRevolution(), m_dt.getTicksPerRotation(), false);
    m_rightTraj = TalonMotionProfileGenerator.generateTalonProfile(m_path_str, "right.csv", m_dt.getMetersPerRevolution(), m_dt.getTicksPerRotation(), false);

    if( m_leftTraj != null && m_rightTraj != null ){
      m_status.forceSetString("Init, starting");
      m_dt.resetMotionProfile();
      m_isDone = false;
      m_dt.startMotionProfile(m_leftTraj, m_rightTraj);
    } else{
      // Something was wrong with the selected profile, just notify and do nothing
      m_isDone = true;
      m_status.forceSetString("Failed to generate profile");
    }

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_status.forceSetString("Execute");

    // Periodically print the status to the console, ONLY if the A button is pressed
    if( m_driver_controller.getAButtonPressed() ){
      m_dt.showInstrumentation();
    }

    m_isDone = m_dt.isMotionProfileFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset the control mode of the talons to be PercentOutput
    // Set the output to 0 (to prevent any oddities when we come back out of MotionProfile mode)
    m_status.forceSetString("Profile end: " + (interrupted ? "INTERRUPTED" : "CLEAN"));
    m_dt.stopMotionProfile();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
