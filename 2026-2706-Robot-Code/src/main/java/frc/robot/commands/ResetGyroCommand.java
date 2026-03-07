package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

//Resets gyro for swerve drive
public class ResetGyroCommand extends Command{
    
    private final SwerveSubsystem m_swerveSubsystem;

    public ResetGyroCommand(SwerveSubsystem swerveSubsystem){
        m_swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    //Reset gyro on initilization of command
    @Override
    public void initialize() {
        m_swerveSubsystem.resetGyro();
    }

    //Only needs to be run once
    @Override
    public boolean isFinished() {
        return true;
    }

}