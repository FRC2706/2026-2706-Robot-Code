package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command{

    private final SwerveSubsystem m_swerveDrive;
    private final DoubleSupplier m_VxSupplier;
    private final DoubleSupplier m_VySupplier;
    private final DoubleSupplier m_omegaSupplier;

    
    public SwerveDriveCommand(SwerveSubsystem swerveDrive, DoubleSupplier VxSupplier, DoubleSupplier VySupplier, DoubleSupplier omegaSupplier) {
        m_swerveDrive = swerveDrive;
        m_VxSupplier = VxSupplier; // Supplier for direction away from the front/driverstation
        m_VySupplier = VySupplier; // Supplier for direction towards the left wall
        m_omegaSupplier = omegaSupplier; // Supplier for rotations counter-clockwise

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        // Initialization logic for the command
    }

    @Override
    public void execute() {
        m_swerveDrive.drive( new Translation2d(
            m_VxSupplier.getAsDouble(),
            m_VySupplier.getAsDouble()),
            m_omegaSupplier.getAsDouble(),
            false // Assuming field-relative control
        );
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should end
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }  
}
