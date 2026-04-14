package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AlignShooterCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;

  // Last computed yaw error (degrees)
  private double lastYawErrorDeg = 0.0;

  private static final double kDeadbandDeg = 1.0; // degrees within which we consider aligned

  // PID for rotation
  private static final double rotationP = 0.05; 

  public AlignShooterCommand(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;    
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // Publish a NetworkTables entry so external tools (Scope/NT viewers) can tell when
    // this command is running. Key: /commands/AlignShooter/Running
    NetworkTableEntry running = NetworkTableInstance.getDefault().getTable("commands").getEntry("AlignShooter/Running");
    running.setBoolean(true);
    System.out.printf("AlignShooterCommand.initialize: desired=%.2f current=%.2f\n", m_swerveSubsystem.getAlignedYaw(), m_swerveSubsystem.getYaw());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Compute yaw error and normalize to [-180, 180]
    double desired = m_swerveSubsystem.getAlignedYaw();
    double current = m_swerveSubsystem.getYaw();
    double yawErrorDeg = desired - current;
    while (yawErrorDeg > 180.0) yawErrorDeg -= 360.0;
    while (yawErrorDeg <= -180.0) yawErrorDeg += 360.0;
    lastYawErrorDeg = yawErrorDeg;

    double rotationVelocity = 0.0;
    if (Math.abs(yawErrorDeg) > kDeadbandDeg) {
      // P-controller in degrees, then convert to radians/sec
      rotationVelocity = yawErrorDeg * rotationP;
      rotationVelocity = Math.toRadians(Math.max(-30.0, Math.min(30.0, rotationVelocity)));
    } else {
      rotationVelocity = 0.0;
    }

    // Rotate the robot (robot-relative)
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), rotationVelocity, false);
    
  }

  // Clear the running flag when the command ends
  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
    NetworkTableEntry running = NetworkTableInstance.getDefault().getTable("commands").getEntry("AlignShooter/Running");
    running.setBoolean(false);
  }

  // End when within the deadband
  @Override
  public boolean isFinished() {
    return Math.abs(lastYawErrorDeg) <= kDeadbandDeg;
  }
}