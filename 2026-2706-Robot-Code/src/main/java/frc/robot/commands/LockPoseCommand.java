package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class LockPoseCommand extends Command {
    private final SwerveSubsystem m_SwerveSubsystem;

    public LockPoseCommand(SwerveSubsystem swerveSubsystem){
        m_SwerveSubsystem = swerveSubsystem;

        addRequirements(m_SwerveSubsystem);
    }

    @Override
    public void initialize() {
        m_SwerveSubsystem.lockPose();
    }

    @Override
    public void execute() {
        m_SwerveSubsystem.lockPose();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
