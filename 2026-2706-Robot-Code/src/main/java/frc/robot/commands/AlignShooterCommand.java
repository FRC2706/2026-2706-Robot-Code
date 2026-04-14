package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignShooterCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;

  private static final double kDeadbandDeg = 1.0; // degrees within which we consider aligned

  // PID for rotation
  private static final double rotationP = 0.05; 

  public AlignShooterCommand(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;    
    addRequirements(swerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check how off we are from the rotation we want
    double yawErrorDeg = m_swerveSubsystem.robotAlignmentYaw - m_swerveSubsystem.getYaw();

    double rotationVelocity = 0;

    if (Math.abs(yawErrorDeg) > kDeadbandDeg){
      rotationVelocity = yawErrorDeg * rotationP;

      //Clamp rotation velocity
      rotationVelocity = Math.toRadians(Math.max(-30, Math.min(30,rotationVelocity)));
    }
    else{
      rotationVelocity = 0;
    }
    
    //Rotate the robot
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), rotationVelocity, false);
    
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.drive(new Translation2d(0, 0), 0, false);
  }

  // Command does not end until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}