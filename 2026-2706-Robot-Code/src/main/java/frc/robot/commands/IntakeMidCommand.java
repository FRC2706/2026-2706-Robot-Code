package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMidCommand extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;

    public IntakeMidCommand(IntakeSubsystem intakeSubsystem){
        m_IntakeSubsystem = intakeSubsystem;

        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.moveIntakeMid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
