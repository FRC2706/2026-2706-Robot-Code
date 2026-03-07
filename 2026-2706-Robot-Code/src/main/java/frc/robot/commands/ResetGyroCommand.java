package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ResetGyroCommand extends Command{
    
    private final SwerveSubsystem m_swerveSubsystem;

    public ResetGyroCommand(SwerveSubsystem swerveSubsystem){
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_swerveSubsystem.resetGyro();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

