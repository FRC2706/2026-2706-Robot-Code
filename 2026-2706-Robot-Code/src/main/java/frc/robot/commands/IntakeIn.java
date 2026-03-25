package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeIn extends Command{
    private final IntakeSubsystem m_IntakeSubsystem;

    public IntakeIn(IntakeSubsystem intakeSubsystem){
        m_IntakeSubsystem = intakeSubsystem;

        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.startIntake();
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return m_IntakeSubsystem.isCurrentNominal();
    }
}
