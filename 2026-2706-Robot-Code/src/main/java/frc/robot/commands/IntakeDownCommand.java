package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDownCommand extends Command {

    private final IntakeSubsystem intake;

    public IntakeDownCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.moveIntakeDown();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
