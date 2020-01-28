/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.Constants.DRIVER_REMOTE_PORT;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveTrainCommand;
import frc.robot.commands.RunProfilePlan;
import frc.robot.subsystems.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Shuffleboard Interactions
  private final ShuffleboardTab m_ProfileTab;
  private final SendableChooser<String> m_pathChooser;

  // The robot's subsystems are defined here
  private final DriveTrain m_drive_train = new DriveTrain();
  
  // The robot's operator interface functionality goes here
  private final XboxController m_driver_controller = new XboxController(DRIVER_REMOTE_PORT);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_pathChooser = new SendableChooser<String>();
    m_pathChooser.addOption("Drive 10", "drive_ten_");
    // Setup the Shuffleboard Tab for testing
    m_ProfileTab = Shuffleboard.getTab("ProfileTest");
    m_ProfileTab.add("Profile to Run", m_pathChooser)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
    m_ProfileTab.add("Run Profile Command", new RunProfilePlan(m_drive_train, m_pathChooser))
      .withWidget(BuiltInWidgets.kCommand);
      
    // Set the default commands for the subsystems
    m_drive_train.setDefaultCommand(new DefaultDriveTrainCommand(m_drive_train, m_driver_controller));

    // Configure the button bindings
    // NOTE -- This should not be called until all the subsystems have been instantiated and the 
    // default commands for them have been set.
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}