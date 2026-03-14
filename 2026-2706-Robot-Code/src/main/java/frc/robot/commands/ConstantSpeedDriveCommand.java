package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

//Drives swerve-chassis using an X and Y translation and rotation
public class ConstantSpeedDriveCommand extends Command{
    private final SwerveSubsystem m_SwerveDrive;
    private final DoubleSupplier m_Vx;
    private final DoubleSupplier m_Vy;
    private final DoubleSupplier m_Omega;
    private final Double m_DriveDeadband;
    private final Double m_AngleDeadband;
    private final Double m_ConstantSpeed;

    public ConstantSpeedDriveCommand(SwerveSubsystem swerveDrive, DoubleSupplier Vx, DoubleSupplier Vy, DoubleSupplier omega, Double driveDeadband, Double angleDeadband, Double constantSpeed) {
        m_SwerveDrive = swerveDrive;
        m_Vx = Vx;
        m_Vy = Vy;
        m_Omega = omega;
        m_DriveDeadband = driveDeadband;
        m_AngleDeadband = angleDeadband;
        m_ConstantSpeed = constantSpeed;

        addRequirements(m_SwerveDrive);
    }

    @Override
    public void execute() {

        //Adjusted Vx, Vy, and Omega for controller deadband
        double m_AdjustedVx = m_Vx.getAsDouble();
        double m_AdjustedVy = m_Vy.getAsDouble();
        double m_AdjustedOmega = m_Omega.getAsDouble();

        //Scale factor to adjust speeds; Makes sure the robot drives at a constant speed even diagonally
        double m_VScaleFactor;

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

        //Calculate the scale factor
        m_VScaleFactor = Math.sqrt(Math.pow(m_AdjustedVx,2) + Math.pow(m_AdjustedVy,2));       
        
        //Make sure speed is not 0 before applying scaling
        if (m_AdjustedVx != 0){
            m_AdjustedVx = m_AdjustedVx / m_VScaleFactor * m_ConstantSpeed;
        }
        if (m_AdjustedVy != 0){
            m_AdjustedVy = m_AdjustedVy / m_VScaleFactor * m_ConstantSpeed;
        }

        //Drive using adjusted values
        m_SwerveDrive.drive( new Translation2d(
            m_AdjustedVx , 
            
            m_AdjustedVy ), 
            
            m_AdjustedOmega * m_SwerveDrive.getMaximumChassisAngularVelocity(), 
            
            true // Assuming field-relative control
        );
    }

    //Driving should never end
    @Override
    public boolean isFinished() {
        return false;
    }
}
