/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.TalonMotionProfileGenerator;

public class RunCTREProfileExample extends CommandBase {
  private final DriveTrain m_dt;
  private final XboxController m_driver_controller;
  private BufferedTrajectoryPointStream m_leftTraj, m_rightTraj;
  private NetworkTableEntry m_status;
  private boolean m_isDone;
  /**
   * Creates a new RunCTREProfileExample.
   */
  public RunCTREProfileExample(DriveTrain dt, XboxController cont) {
    m_dt = dt;
    m_driver_controller = cont;
    m_isDone = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dt);
    m_status = Shuffleboard.getTab("ProfileTest").add("CTRE Profile Status", "Not Run Yet").withWidget(BuiltInWidgets.kTextView).getEntry();
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
   
    m_leftTraj = TalonMotionProfileGenerator.generateExampleProfile();
    m_rightTraj = TalonMotionProfileGenerator.generateExampleProfile();

    if( m_leftTraj != null && m_rightTraj != null ){
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
    // Periodically print the status to the console, ONLY if the A button is pressed
    if( m_driver_controller.getAButtonPressed() ){
      m_dt.showInstrumentation();
    }
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
