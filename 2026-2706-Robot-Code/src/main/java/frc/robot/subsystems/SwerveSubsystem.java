package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports necessary to create SwerveDrive object
import java.io.File;
import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase{

    double maximumSpeed = 3;

    // Swerve drive object
    private final SwerveDrive swerveDrive; 
    // Field2d for SmartDashboard / Field widget
    private final Field2d m_field = new Field2d();
    
    
    // Provide swerve configuration file as arguement
    public SwerveSubsystem(File swerveJsonDirectory){
        // If the deploy directory used at runtime doesn't contain the expected swerve
        // config files, fall back to the repository's src/main/deploy/swerve folder so
        // local simulation uses the up-to-date JSON files during development.
        File checkedSwerveDir = swerveJsonDirectory;
        try {
            File deploySwerve = new File(Filesystem.getDeployDirectory(), "swerve");
            File srcSwerve = new File("src/main/deploy/swerve");
            if (deploySwerve.exists() && new File(deploySwerve, "swervedrive.json").exists()) {
                checkedSwerveDir = deploySwerve;
            } else if (srcSwerve.exists() && new File(srcSwerve, "swervedrive.json").exists()) {
                checkedSwerveDir = srcSwerve;
                System.out.println("[DEBUG] Using src/main/deploy/swerve as swerve config directory for simulation.");
            } else {
                System.out.println("[DEBUG] No swerve config found in deploy or src/main/deploy; using provided directory: " + swerveJsonDirectory.getAbsolutePath());
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        
        // Set up starting position depending on alliance for odometry
        boolean redAlliance = isRedAlliance();
        Pose2d startingPose;

        // Set the verbosity of the telemetry.  HIGH is good for debugging, but may cause performance issues.  Adjust as needed.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO; 

        // TODO: Set up different starting positions
        if (redAlliance){
            // Flip in red alliance
            startingPose =  new Pose2d(new Translation2d(16, 4), Rotation2d.fromDegrees(180)); 
        }
        else{
            // Blue alliance
            startingPose = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(0)); 
        }
        
        // Debug: print the chosen swerve config directory and its swervedrive.json so we can
        // verify which JSONs are being loaded at runtime.
        try {
            System.out.println("[DEBUG] Chosen swerve config directory: " + checkedSwerveDir.getAbsolutePath());
            File swervedriveFile = new File(checkedSwerveDir, "swervedrive.json");
            if (swervedriveFile.exists()) {
                System.out.println("[DEBUG] swervedrive.json contents:");
                java.util.Scanner s = new java.util.Scanner(swervedriveFile).useDelimiter("\\A");
                System.out.println(s.hasNext() ? s.next() : "<empty>");
                s.close();
            } else {
                System.out.println("[DEBUG] swervedrive.json not found in chosen directory");
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }

        // Parse swerve configurations and create swerve drive object
        try{
            swerveDrive = new SwerveParser(checkedSwerveDir).createSwerveDrive(maximumSpeed, startingPose);
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        // Configure Swerve Drive
    swerveDrive.setHeadingCorrection(true); // Turn on to correct heading
    // Disable low-level angular velocity correction flags so we don't attempt IMU-based
    // compensation when an IMU isn't present in simulation. Use setAngularVelocityCompensation
    // to enable with proper parameters and a valid IMU.
    swerveDrive.angularVelocityCorrection = false;
    swerveDrive.autonomousAngularVelocityCorrection = true;
        swerveDrive.setCosineCompensator(false); // Turn on to automatically slow or speed up swerve modules that should be close to their desired state in theory
            // Disable angular velocity compensation when IMU may be unavailable in simulation to avoid NPEs.
            // If you have a working IMU in sim/hardware, set this to true and tune the parameters.
            swerveDrive.setAngularVelocityCompensation(false, true, 0.1); // Tune to compensate for angular skew in movement
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // Turn on to periodcally synchronize absolute encoders and motor encoders during periods without movement
        swerveDrive.synchronizeModuleEncoders();

        // Publish Field2d so SmartDashboard / sim can display the robot pose
        try {
            SmartDashboard.putData("Field", m_field);
        } catch (Exception e) {
            // Ignore if SmartDashboard isn't available at construction time
        }

        //setupPathPlanner();
    }
    
    public void setupPathPlanner(){
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
                        new PIDConstants(0.0025, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.0025, 0.0, 0.0) // Rotation PID constants
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
    }

    @Override
    public void periodic(){
        updateOdometry();
        // Update the Field2d with the latest odometry pose so dashboard shows robot
        try {
            m_field.setRobotPose(getPose());
        } catch (Exception e) {
            // Ignore if pose is not ready yet
        }
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

    public void drive(ChassisSpeeds speeds)
    {
        swerveDrive.drive(speeds);
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
    public void resetGyro()
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
    
    //Autonmous Path Following Commands
    public Command getAutounomousCommand(String pathName){
        return new PathPlannerAuto(pathName);
    }
    
    
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
  // public Command driveForward()
  public Command driveForward(){
    return driveToPose(
        new Pose2d(
            getPose().getTranslation().plus(new Translation2d(1.0, 0.0)), // Drive forward 1 meter
            getPose().getRotation()
        )
    );
  }
    
    /**
     * Return robot relative velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    
}
