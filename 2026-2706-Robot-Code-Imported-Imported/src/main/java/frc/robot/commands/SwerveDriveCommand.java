package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

<<<<<<< HEAD:2026-2706-Robot-Code-Imported-Imported/src/main/java/frc/robot/commands/SwerveDriveCommand.java
=======
//Drives swerve-chassis using an X and Y translation and rotation
>>>>>>> 2952a1b5c6fa8d5cd87c2c491b16487442fe7635:2026-2706-Robot-Code/src/main/java/frc/robot/commands/SwerveDriveCommand.java
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
<<<<<<< HEAD:2026-2706-Robot-Code-Imported-Imported/src/main/java/frc/robot/commands/SwerveDriveCommand.java
            m_AdjustedVx*m_SwerveDrive.getMaximumChassisVelocity(), 
            
            m_AdjustedVy*m_SwerveDrive.getMaximumChassisVelocity()), 
            
            m_AdjustedOmega*m_SwerveDrive.getMaximumChassisAngularVelocity()*0.5, 
=======
            m_AdjustedVx * m_SwerveDrive.getMaximumChassisVelocity(), 
            
            m_AdjustedVy * m_SwerveDrive.getMaximumChassisVelocity()), 
            
            m_AdjustedOmega * m_SwerveDrive.getMaximumChassisAngularVelocity(), 
>>>>>>> 2952a1b5c6fa8d5cd87c2c491b16487442fe7635:2026-2706-Robot-Code/src/main/java/frc/robot/commands/SwerveDriveCommand.java
            
            true // Assuming field-relative control
        );
    }

<<<<<<< HEAD:2026-2706-Robot-Code-Imported-Imported/src/main/java/frc/robot/commands/SwerveDriveCommand.java
    @Override
    public boolean isFinished() {
        // Return true when the command should end
=======
    //Driving should never end
    @Override
    public boolean isFinished() {
>>>>>>> 2952a1b5c6fa8d5cd87c2c491b16487442fe7635:2026-2706-Robot-Code/src/main/java/frc/robot/commands/SwerveDriveCommand.java
        return false;
    }
}
