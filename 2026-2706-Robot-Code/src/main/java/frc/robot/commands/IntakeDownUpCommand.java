package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDownUpCommand extends Command {

    private final IntakeSubsystem intake;

    public IntakeDownUpCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.moveIntakeDown();
    }

    @Override
    public void end(boolean interrupted) {
        intake.moveIntakeUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
