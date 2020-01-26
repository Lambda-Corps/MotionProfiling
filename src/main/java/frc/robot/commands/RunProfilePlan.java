/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class RunProfilePlan extends CommandBase {
  private final DriveTrain m_dt;
  private final SendableChooser<String> m_chooser;
  private final String PROFILE_PATH_DIR = "/home/lvuser/deploy/paths/";
  private String m_path_str = "";
  /**
   * Creates a new RunProfilePlan.
   */
  public RunProfilePlan(DriveTrain dt, SendableChooser<String> chooser) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dt = dt;
    m_chooser = chooser;
    addRequirements(m_dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_path_str = PROFILE_PATH_DIR + m_chooser.getSelected();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
