package frc.robot.subsystems;
import static frc.robot.subsystems.ShooterStateMachine.ShooterModes.*;
import static frc.robot.subsystems.ShooterStateMachine.States.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShooterStateMachine.ShooterModes;
import frc.robot.subsystems.ShooterStateMachine.States;


public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  
  public ShooterSubsystem() {}

  /**
  
   * @return 
   */
  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
         
        });
  }


  @Override
  public void periodic() {
   
  }

  @Override
  public void simulationPeriodic() {
    
  }
}