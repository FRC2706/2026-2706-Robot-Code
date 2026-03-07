// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.commands.PhotonAlignToTargetCommand;
//import frc.robot.commands.AlignToTargetCommand;

// Pathplanner testing
import frc.robot.subsystems.AutoPlans;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final PhotonSubsystem m_photonSubsystem = new PhotonSubsystem(); // name from PhotonVision/config
  // Pathplanner testing
  private final AutoPlans m_autoPlans = new AutoPlans();
  private final SendableChooser<Command> autoChooser;
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    DriverStation.silenceJoystickConnectionWarning(false);

    m_swerveSubsystem.setDefaultCommand(
       new SwerveDriveCommand(
          m_swerveSubsystem,
          () -> -m_driverController.getLeftY(), // Forward/backward
          () -> -m_driverController.getLeftX(), // Left/right
          () -> -m_driverController.getRightX(),0.1,0.1)
    );

    // NamedCommands.registerCommand("test", new AlignToTargetCommand(m_swerveSubsystem, m_photonVision));

    // Configure PathPlanner/AutoBuilder now that the swerve subsystem exists
    // This will configure AutoBuilder using the subsystem-provided callbacks.
    m_swerveSubsystem.setupPathPlanner();

    // Now that AutoBuilder is configured, create autos and the chooser
    autoChooser = AutoBuilder.buildAutoChooser("Drive Forward Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    PhotonAlignToTargetCommand alignCommand = new PhotonAlignToTargetCommand(m_photonSubsystem, m_swerveSubsystem);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
       // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.rightTrigger().whileTrue(alignCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return the command selected on the SendableChooser (built by AutoBuilder).
    Command selected = autoChooser.getSelected();
    

    //If the chooser has no selection, fall back to the AutoPlans default.
    if (selected != null) {
      return selected;
    }
    return m_autoPlans.getAutonomousCommand(0);

  }
}
