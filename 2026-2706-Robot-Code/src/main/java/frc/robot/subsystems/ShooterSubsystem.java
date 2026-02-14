package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor1;
  private SparkMax shooterMotor2;
  private SparkMax feederMotor; // CANNOT be faster than shooterMotor RPM
  private SparkMax indexerMotor;
  
  public ShooterSubsystem() {}
  /**
  
   * @return 
   */
  public void stop(){
        System.out.println("stop cmd called");
        shooterMotor1.stopMotor(); 
        shooterMotor2.stopMotor(); 
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
  }


  @Override
  public void periodic() {
   
  }

  @Override
  public void simulationPeriodic() {
    
  }
}