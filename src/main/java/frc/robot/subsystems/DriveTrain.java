/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.InstrumentProfile;

import static frc.robot.Constants.*;
public class DriveTrain extends SubsystemBase {
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1; // TODO, tune this deadband to actually work with robot

  private final TalonSRX m_rightLeader, m_rightFollower;
  private final TalonSRX m_leftLeader, m_leftFollower;

  private final DoubleSolenoid m_gearbox;


  /**********************************************************************************
   * Configuration Values for Motion Profiling
   **********************************************************************************/
  private PeriodicProcessor m_profileProcessor;

  /** TODO : Verify these numbers for the target drive train being tested
   * 512 encoder ticks per axle rotation * 36/12 * 50/34 (gearing) = 2259 encoder ticks per wheel rotation
   * Wheel diameter is 16cm - pi * d (circumference) - Wheel Circumference = 50.27cm
   * 2259 / 50.27 = 45 ticks/cm
   * 45/cm = 4500/m
   */
  private double METERS_PER_ROTATION = .5027; // *
  private int TICKS_PER_ROTATION = 512; // Taken from the Grayhill Spec Sheet
   /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    
    // TODO -- Finish with phoenix tuner to determine inversion, sensor phase
    // and all the rest of the Bring-Up steps for the Talon
    m_rightLeader = new TalonSRX(RIGHT_TALON_LEADER);
    m_rightLeader.configFactoryDefault();
    m_rightFollower = new TalonSRX(RIGHT_TALON_FOLLOWER);
    m_rightFollower.configFactoryDefault();
    m_rightFollower.follow(m_rightLeader);

    // Phoenix Tuner showed left side needs to be inverted
    m_leftLeader  = new TalonSRX(LEFT_TALON_LEADER);
    m_leftLeader.configFactoryDefault();
    m_leftLeader.setInverted(true);
    m_leftFollower = new TalonSRX(LEFT_TALON_FOLLOWER);
    m_leftFollower.configFactoryDefault();
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_leftFollower.follow(m_leftLeader);

    // Setup the Talons for encoder feedback
    m_rightLeader.setSensorPhase(true);
    m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_leftLeader.setSelectedSensorPosition(0, 0, 0);
    m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_rightLeader.setSelectedSensorPosition(0, 0, 0);

    // Scheduler loop, as well as generated profiles are 20 ms.  Set the status frame
    // to half the period.  NOTE: If generated profiles use a different time step, then
    // this should be updated to be half of that value.
    m_rightLeader.configMotionProfileTrajectoryPeriod(20, 10);
    m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);
    m_leftLeader.configMotionProfileTrajectoryPeriod(20, 10);
    m_rightLeader.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

    m_gearbox = new DoubleSolenoid(0, 1);

  }

  public void teleop_drive(double left, double right){
    curvature_drive_imp(left, right, true);
  }

  /* private void tank_drive_imp(double left_speed, double right_speed){
    m_leftLeader.set(ControlMode.PercentOutput, left_speed);
    m_rightLeader.set(ControlMode.PercentOutput, right_speed);
  }
  */
  private void curvature_drive_imp(double xSpeed, double zRotation, boolean isQuickTurn) {

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);

    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, m_deadband);

    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    m_leftLeader.set(ControlMode.PercentOutput, leftMotorOutput);
    m_rightLeader.set(ControlMode.PercentOutput, rightMotorOutput);

  }

  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetMotionProfile(){
    m_rightLeader.clearMotionProfileTrajectories();
    m_leftLeader.clearMotionProfileTrajectories();
  }

  public void startMotionProfile(BufferedTrajectoryPointStream pointsLeft, BufferedTrajectoryPointStream pointsRight){
    m_profileProcessor = new PeriodicProcessor();
    m_profileProcessor.run();

    // Start the profiles, waiting for 10 points to arrive at the talon before starting.
    m_leftLeader.startMotionProfile(pointsLeft, 10, ControlMode.MotionProfile);
    m_rightLeader.startMotionProfile(pointsRight, 10, ControlMode.MotionProfile);
    
  }

  public boolean isMotionProfileFinished(){
    return (m_leftLeader.isMotionProfileFinished() && m_rightLeader.isMotionProfileFinished() );
  }

  public void stopMotionProfile(){
    m_profileProcessor.shutdown();
    resetMotionProfile();

    m_leftLeader.set(ControlMode.PercentOutput, 0);
    m_rightLeader.set(ControlMode.PercentOutput, 0);
  }

  public double getMetersPerRevolution(){
    return METERS_PER_ROTATION;
  }

  public int getTicksPerRotation() {
    return TICKS_PER_ROTATION;
  }
  
  // Private inner class used to spawn a thread with a singular purpose to process the buffer from the high level
  // API buffer, to the low level controller buffer.
  private class PeriodicProcessor implements Runnable {
    private boolean bisDone = false;
    public void run() {
      while(!bisDone){
        m_leftLeader.processMotionProfileBuffer();
        m_rightLeader.processMotionProfileBuffer();
      }
    }

    public void shutdown(){
      bisDone = true;
    }
  }

  public void showInstrumentation() {
    InstrumentProfile.loop(true, m_leftLeader, m_rightLeader);
  }
}
