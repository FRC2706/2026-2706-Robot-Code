// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// Imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.UtilityConstants.VisionConstants;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.DoubleSupplier;

// Class
/** An example command that uses an example subsystem. */
public class PhotonAlignToTargetCommand extends Command {
  private final PhotonSubsystem m_photonSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  
  // Goal distance from the AprilTag (in meters)
  private final double goalDistance;
  
  // PID/Proportional gains for distance control
  private static final double kDistanceGain = 0.5; // Proportional gain for forward/backward movement
  private static final double kYawGain = 0.05; // Proportional gain for rotation

  //Deadband for if the robot should continue rotating -- in degrees
  private static final double kDeadband = 1;

  //Swerve drive variables
  private final DoubleSupplier m_Vx;
  private final DoubleSupplier m_Vy;
  private final DoubleSupplier m_Omega;
  private final Double m_DriveDeadband;
  private final Double m_AngleDeadband;

  /**
   * Creates a new PhotonAlignToTargetCommand.
   *
   * @param photonSubsystem The PhotonVision subsystem
   * @param swerveSubsystem The swerve drive subsystem
   */
  public PhotonAlignToTargetCommand(PhotonSubsystem photonSubsystem, SwerveSubsystem swerveSubsystem,DoubleSupplier Vx, DoubleSupplier Vy, DoubleSupplier omega, Double driveDeadband, Double angleDeadband) {
    m_photonSubsystem = photonSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_Vx = Vx;
    m_Vy = Vy;
    m_Omega = omega;
    m_DriveDeadband = driveDeadband;
    m_AngleDeadband = angleDeadband;
    this.goalDistance = VisionConstants.kGoalDistance;
    
    addRequirements(photonSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PhotonAlignToTargetCommand started with goal distance: " + goalDistance + " m");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Adjusted Vx, Vy, and Omega for controller deadband
    double m_AdjustedVx = m_Vx.getAsDouble();
    double m_AdjustedVy = m_Vy.getAsDouble();
    double m_AdjustedOmega = m_Omega.getAsDouble();

    //Applying deadbands
    if (Math.abs(m_AdjustedVx) < m_DriveDeadband){
        m_AdjustedVx = 0;
    }
    if (Math.abs(m_AdjustedVy) < m_DriveDeadband){
        m_AdjustedVy = 0;
    }
    if (Math.abs(m_AdjustedOmega) < m_AngleDeadband){
        m_AdjustedOmega = 0;
    }

    // Check if we can see the target
    if (!m_photonSubsystem.hasTarget()) {
      System.out.println("No target detected");
      // Drive the robot as if not aligning target
      m_swerveSubsystem.drive(new Translation2d(m_AdjustedVx * m_swerveSubsystem.getMaximumChassisVelocity(), 
                              m_AdjustedVy * m_swerveSubsystem.getMaximumChassisVelocity()), 
                              m_AdjustedOmega * m_swerveSubsystem.getMaximumChassisAngularVelocity(), 
                              true);
      return;
    }
    
    // Get yaw error (left/right)
    double yawError = m_photonSubsystem.getYaw();
    
    //Check if the yawError is in the deadband
    if (Math.abs(yawError) > kDeadband){
      // Calculate rotation velocity: proportional to yaw error
      // Negative yaw = target to the left, positive yaw = target to the right
      // Convert yaw from degrees to radians for rotation velocity (rad/s)
      m_AdjustedOmega = -Math.toRadians(yawError) * kYawGain;
      
      // Clamp rotation velocity (in rad/s)
      m_AdjustedOmega = Math.max(-0.3, Math.min(0.3, m_AdjustedOmega));
    }
    else{
      //Prevent rotation if the robot is within the deadband
      m_AdjustedOmega = 0;
    }

    m_swerveSubsystem.drive(new Translation2d(m_AdjustedVx * m_swerveSubsystem.getMaximumChassisVelocity(), 
                              m_AdjustedVy * m_swerveSubsystem.getMaximumChassisVelocity()), 
                              m_AdjustedOmega, 
                              true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    //m_swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
    System.out.println("PhotonAlignToTargetCommand ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}