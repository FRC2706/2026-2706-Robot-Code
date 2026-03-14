package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

public class StopConstantSpeedDriveCommand extends Command{
    
    private final SwerveSubsystem m_SwerveSubsystem;

    public StopConstantSpeedDriveCommand(SwerveSubsystem swerveSubsystem){

        m_SwerveSubsystem = swerveSubsystem;

        addRequirements(m_SwerveSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
