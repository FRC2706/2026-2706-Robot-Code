package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports necessary to create SwerveDrive object
import java.io.File;
import java.util.function.DoubleSupplier;

//Useful imports for swerve
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

// Imports for pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase{

    double maximumSpeed = 3;

    // Swerve drive object
    private final SwerveDrive swerveDrive; 
    // Field2d visualization is now centralized in AutoPlans; SwerveSubsystem will
    // update the shared field via AutoPlans.setMainFieldRobotPose(...)

    // Provide swerve configuration file as arguement
    public SwerveSubsystem(File swerveJsonDirectory){
        
        // Set up starting position depending on alliance for odometry
        boolean redAlliance = isRedAlliance();
        Pose2d startingPose;

        // Set the verbosity of the telemetry.  HIGH is good for debugging, but may cause performance issues.  Adjust as needed.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO; 

        // TODO: Set up different starting positions
        if (redAlliance){
            // Units are in meters
            startingPose =  new Pose2d(new Translation2d(16, 4), Rotation2d.fromDegrees(180));
        }
        else{
            // Flip for blue alliance
            startingPose = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(180));
        }
        
        // Parse swerve configurations and create swerve drive object
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, startingPose);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        // Configure Swerve Drive
        // Some configurations rely on an IMU being present in the underlying swervelib SwerveDrive.
        // In simulation or if the IMU isn't available, these calls can throw a NullPointerException
        // (see logs). Wrap configuration in a try/catch and disable angular-velocity-based
        // compensations if they fail so the robot code keeps running.
        try {
            swerveDrive.setHeadingCorrection(true); // Turn on to correct heading
            swerveDrive.setCosineCompensator(true); // Turn on to automatically slow or speed up swerve modules that should be close to their desired state in theory
            swerveDrive.angularVelocityCorrection = true;
            swerveDrive.autonomousAngularVelocityCorrection = true;
            swerveDrive.setAngularVelocityCompensation(true, true, 0.1); // Tune to compensate for angular skew in movement
            swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // Turn on to periodcally synchronize absolute encoders and motor encoders during periods without movement
            swerveDrive.synchronizeModuleEncoders();
        } catch (Throwable t) {
            // Defensive: disable IMU/ang. vel. dependent features and continue running
            System.err.println("Warning: failed to configure angular-velocity/IMU features in SwerveDrive. Disabling those features.\n" + t);
            try {
                swerveDrive.angularVelocityCorrection = false;
                swerveDrive.autonomousAngularVelocityCorrection = false;
                // Attempt to disable compensation; library method may still throw, so ignore exceptions
                try {
                    swerveDrive.setAngularVelocityCompensation(false, false, 0.0);
                } catch (Throwable ignore) {
                    // ignore
                }
            } catch (Throwable ignore) {
                // ignore
            }
        }
    
    // NOTE: Do not call setupPathPlanner() here; it is invoked from RobotContainer to avoid
    // double-configuration of AutoBuilder. PathPlanner's AutoBuilder.configure(...) must be
    // called exactly once during program startup.
    // AutoPlans publishes the shared Field2d; nothing to publish here.
    }

    //Sets up pathplanner
    public void setupPathPlanner()
    {
        // PATH PLANNER FILE SETUP
        RobotConfig config;

        try{
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose, // Pass method supplying robot pose
                this::resetOdometry, // Pass method reseting odometry
                this::getRobotVelocity, // Pass method supplying robot relative chassis
                (speedsRobotRelative, moduleFeedForwards) -> {this.drive(speedsRobotRelative);}, // Pass method that will drive the robot -- only robot relative chassis speeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(4.9, 0.0003, 0.015), // Translation PID constants
                        new PIDConstants(0.0025, 0.0002, 0.0001) // Rotation PID constants
                ),  
                config, // Pass on the config
                () -> isRedAlliance(), // Check which alliance the robot is on
                this // Reference this subsystem
            );
        } catch (Exception e) 
        {
            // Handle exception as needed
            e.printStackTrace();
        }

        //Prevents issue with java
        PathfindingCommand.warmupCommand().schedule();
    }

    @Override
    public void periodic(){
        updateOdometry();
    }

   /**
   * Controls the drivebase.  Takes a Translation2d and a rotation rate, and calculates and commands module states accordingly. 
   *
   * @param translation   the linear velocity of the robot, in meters per second. In robot-relative mode, 
   *                      positive x is torwards the bow (front) and positive y is torwards port (left).  
   *                      In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  
   * 
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        try {
            swerveDrive.drive(translation,
                              rotation,
                              fieldRelative,
                              false); // Open loop is disabled since it shouldn't be used most of the time.
        } catch (Throwable t) {
            // Defensive: if the underlying swervelib throws (for example, because IMU is null),
            // log once and try to disable IMU-dependent behavior to prevent further crashes.
            System.err.println("SwerveSubsystem: drive() failed, disabling IMU-dependent features. Exception: " + t);
            try {
                swerveDrive.angularVelocityCorrection = false;
                swerveDrive.autonomousAngularVelocityCorrection = false;
                try { swerveDrive.setAngularVelocityCompensation(false, false, 0.0); } catch (Throwable ignore) {}
            } catch (Throwable ignore) {}
        }
    }

    //Controls the drivebase using ChassisSpeeds -- primarily for PathPlanner
    public void drive(ChassisSpeeds speeds)
    {
        try {
            swerveDrive.drive(speeds);
        } catch (Throwable t) {
            System.err.println("SwerveSubsystem: drive(ChassisSpeeds) failed, disabling IMU-dependent features. Exception: " + t);
            try {
                swerveDrive.angularVelocityCorrection = false;
                swerveDrive.autonomousAngularVelocityCorrection = false;
                try { swerveDrive.setAngularVelocityCompensation(false, false, 0.0); } catch (Throwable ignore) {}
            } catch (Throwable ignore) {}
        }
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
     * method.  
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry( Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    //Resets the gyro angle to zero and resets odometry 
    public void resetGyro()
    {
        swerveDrive.zeroGyro();
    }

    // Resets the encoders -- should be used to manually reset robot (i.e after autonomous)
    public void resetDriveEncoders(){
        swerveDrive.resetDriveEncoders();
    }

    //Gets the current pose (position and rotation) of the robot, as reported by odometry.
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    //Gets the heading (yaw) of the robot from the odometry
    public Rotation2d getOdometryHeading(){
        return swerveDrive.getOdometryHeading();
    }

    // Updates the odometry; Should be run periodically
    public void updateOdometry(){
        swerveDrive.updateOdometry();
        // Update field visualization with the latest pose
        try {
            frc.robot.subsystems.AutoPlans.setMainFieldRobotPose(swerveDrive.getPose());
        } catch (Throwable ignore) {
            // If the field can't be updated for any reason, ignore to avoid spamming logs
        }
    }

    // Forces the drive train to not move by pointing all the swerve modueles to the center of the robot
    public void lockPose(){
        swerveDrive.lockPose();
    }

    // Check if the current alliance is the red alliance. Defaults to being on red alliance
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
    
    //Return robot relative velocity
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    //Return field relative velocity
    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }

    //Autonmous Path Following Commands
    public Command getAutounomousCommand(String pathName){
        return new PathPlannerAuto(pathName);
    }
    
    //For pathplanner
    public Command driveToPose(Pose2d pose)
    {
        // Create the constraints to use while pathfinding  
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumChassisVelocity(), 4.0,
            swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                        );
    }
}