package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignShooterCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;

  private static final double kDeadbandDeg = 1.0; // degrees within which we consider aligned

  // PID for rotation
  private static final double rotationP = 0.1; 

  public AlignShooterCommand(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;    
    addRequirements(swerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //lock swerve yaw between -180 and 180
    double swerveYaw = m_swerveSubsystem.getYaw(); 

    swerveYaw = swerveYaw % 360;
    if (swerveYaw < -180){
      swerveYaw = 360 + swerveYaw;
    }
    else if (swerveYaw > 180){
      swerveYaw = -360 + swerveYaw;
    }

    // Check how off we are from the rotation we want
    double yawErrorDeg = m_swerveSubsystem.robotAlignmentYaw - swerveYaw;

    //Normalize yawError
    yawErrorDeg %= 360;
    if (yawErrorDeg < -180){
      yawErrorDeg = 360 + yawErrorDeg;
    }
    else if (yawErrorDeg > 180){
      yawErrorDeg = -360 + yawErrorDeg;
    }

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
    m_swerveSubsystem.drive(new Translation2d(0.0, 0.0), rotationVelocity, true);
    
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends
    m_swerveSubsystem.drive(new Translation2d(0, 0), 0, true);
  }

  // Command does not end until interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}