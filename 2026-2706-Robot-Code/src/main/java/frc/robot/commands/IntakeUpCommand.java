package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeUpCommand extends Command {

    private final IntakeSubsystem intake;

    public IntakeUpCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.moveIntakeUp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}