// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;


import frc.robot.commands.IntakeDownUpCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.RunIntakeCommandReversed;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AutoSelectorKnobSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystem
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AutoSelectorKnobSubsystem m_AutoSelectorKnobSubsystem = new AutoSelectorKnobSubsystem();

  // Controller
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
      configureButtonBindings();
  }

  private void configureButtonBindings() {

        // Toggle intake ON/OFF
        JoystickButton intakeToggleButton =
            new JoystickButton(driverController, XboxController.Button.kA.value);

        // Run intake in reverse while held
        JoystickButton reverseButton =
            new JoystickButton(driverController, XboxController.Button.kB.value);

          // Toggle intake to go up and down
        JoystickButton intakeDownUpButton =
            new JoystickButton(driverController, XboxController.Button.kX.value);

        intakeDownUpButton.toggleOnTrue(new IntakeDownUpCommand(intakeSubsystem));
        intakeToggleButton.toggleOnTrue(new RunIntakeCommand(intakeSubsystem));
        reverseButton.whileTrue(new RunIntakeCommandReversed(intakeSubsystem));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int mode = m_AutoSelectorKnobSubsystem.getAutoMode();
    System.out.println("Auto Mode = " + mode); // debug print
    
    switch (mode) {
      case 0:
        return null; // do nothing
      case 1:
        return new PrintCommand("1");
        //DriveDistance(39, 0.3, m_robotDrive);
      case 2:
        return new PrintCommand("2");
        //DriveTimed(2.0, 0.3, m_robotDrive);
      case 3:
        return new PrintCommand("3");
        //null;
      case 4:
        return new PrintCommand("4");
        //null;
      case 5:
        return new PrintCommand("5");
        //null;
      case 6:
        return new PrintCommand("6");
        //null;
      case 7:
        return new PrintCommand("7");
        //null;
      case 8:
        return new PrintCommand("8");
        //null;
      case 9:
        return new PrintCommand("9");
        //null;
      case 10:
        return new PrintCommand("10");
        //null;
      case 11:
        return new PrintCommand("11");
        //null;
      default:
        return null;
    }
  }
}
