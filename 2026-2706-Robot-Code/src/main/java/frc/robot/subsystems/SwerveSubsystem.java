package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports necessary to create SwerveDrive object
import java.io.File;
import java.util.function.DoubleSupplier;

import org.dynd4j.geometry.Rotation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// Imports necessary to create telemetry
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// For driving
import swervelib.math.SwerveMath;

// Imports for pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;


public class SwerveSubsystem extends SubsystemBase{

    double maximumSpeed = Units.feetToMeters(4.5);

    // Swerve drive object
    private final SwerveDrive swerveDrive; 
    
    
    // Provide swerve configuration file as arguement
    public SwerveSubsystem(File swerveJsonDirectory){
        
        // Set up starting position depending on alliance for odometry
        boolean blueAlliance = isRedAlliance();
        Pose2d startingPose;
        if (blueAlliance){
            // Units are in meters
            startingPose =  new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(0));
        }
        else{
            // Flip for red alliance
            startingPose = new Pose2d(new Translation2d(16, 4), Rotation2d.fromDegrees(180));
        }
        
        // Parse swerve configurations and create swerve drive object
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, startingPose);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        // Configure Swerve Drive
        swerveDrive.setHeadingCorrection(false); // Turn on to correct heading
        swerveDrive.setCosineCompensator(false); // Turn on to automatically slow or speed up swerve modules that should be close to their desired state in theory
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Tune to compensate for angular skew in movement
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1); // Turn on to periodcally synchronize absolute encoders and motor encoders during periods without movement
        
        setupPathPlanner();
    }

    public void setupPathPlanner(){
        // TODO: PATH PLANNER FILE SETUP
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

    // TODO: AUTO TEAM BUILDER SETUP
        /** 
        AutoBuilder.configure(
            this::getPose, // Pass method supplying robot pose
            this::resetOdometry, // Pass method reseting odometry
            null, // Pass method supplying robot relative chassis
            null, // Pass method that will drive the robot -- only robot relative chassis speeds
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),  
            config, 
            this::isRedAlliance, 
            null);*/
    }

    @Override
    public void periodic(){
        updateOdometry();
    }

   /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                        rotation,
                        fieldRelative,
                        false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
    {
        return run(() -> {
            swerveDrive.drive(
                SwerveMath.scaleTranslation(
                    new Translation2d(
                        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
                    ),
                    0.8
                ),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                true,
                false
            );
        });
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
        //resetOdometry();
    }

    // Resets the encoders -- should be used to manually reset robot (i.e after autonomous)
    public void resetDriveEncoders(){
        swerveDrive.resetDriveEncoders();
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    /**
     * Gets the heading (yaw) of the robot from the odometry
     * 
     * @return The robot's heading in Rotation2d
     */
    public Rotation2d getOdometryHeading(){
        return swerveDrive.getOdometryHeading();
    }

    // Updates the odometry; Should be run periodically
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }

    // Forces the drive train to not move by pointing all the swerve modueles to the center of the robot
    public void lockPose(){
        swerveDrive.lockPose();
    }

    // Check if the current alliance is the red alliance
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()){
            if (alliance.get() == DriverStation.Alliance.Red){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            return false;
        }
    }

    /**
     * Return the swerve drive's maximum chassis angular velocity (rad/s).
     * Delegates to the underlying swervelib SwerveDrive instance.
     */
    public double getMaximumChassisAngularVelocity() {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

    /**
     * (Optional helper) Return the swerve drive's maximum chassis linear velocity (m/s).
     */
    public double getMaximumChassisVelocity() {
        return swerveDrive.getMaximumChassisVelocity();
    }


}