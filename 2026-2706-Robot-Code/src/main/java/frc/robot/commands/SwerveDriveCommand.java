package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveCommand extends Command{
    private final SwerveSubsystem m_SwerveDrive;
    private final DoubleSupplier m_Vx;
    private final DoubleSupplier m_Vy;
    private final DoubleSupplier m_Omega;
    private final Double m_DriveDeadband;
    private final Double m_AngleDeadband;

    public SwerveDriveCommand(SwerveSubsystem swerveDrive, DoubleSupplier Vx, DoubleSupplier Vy, DoubleSupplier omega, Double driveDeadband, Double angleDeadband) {
        m_SwerveDrive = swerveDrive;
        m_Vx = Vx;
        m_Vy = Vy;
        m_Omega = omega;
        m_DriveDeadband = driveDeadband;
        m_AngleDeadband = angleDeadband;

        addRequirements(m_SwerveDrive);
    }

    @Override
    public void initialize() {
        // Initialization logic for the command
    }

    @Override
    public void execute() {

        //Adjusted Vx, Vy, and Omega for controller deadband
        double m_AdjustedVx = m_Vx.getAsDouble();
        double m_AdjustedVy = m_Vy.getAsDouble();
        double m_AdjustedOmega = m_Omega.getAsDouble();

        //Applying deadbands
        if (Math.abs(m_AdjustedVx) < m_DriveDeadband){
            m_AdjustedVx = 0;
        }
        if (Math.abs(m_AdjustedVy) < m_DriveDeadband){
            m_AdjustedVy = 0;
        }
        if (Math.abs(m_AdjustedOmega) < m_AngleDeadband){
            m_AdjustedOmega = 0;
        }
        
        //Drive using adjusted values
        m_SwerveDrive.drive( new Translation2d(
            m_AdjustedVx, 
            
            m_AdjustedVy), 
            
            m_AdjustedOmega, 
            
            true // Assuming field-relative control
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
