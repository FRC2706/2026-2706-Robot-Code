package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.config.*;
import java.util.function.BooleanSupplier;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor1;
  private SparkMax shooterMotor2;
  private SparkMax feederMotor; // CANNOT be faster than shooterMotor RPM
  private SparkMax indexerMotor;

    private static ShooterSubsystem shooter;
    public static ShooterSubsystem getInstance() {
        if (shooter == null)
            shooter = new ShooterSubsystem();
        return shooter;
    }
  
  public ShooterSubsystem() {

      //shooterMotor1 = new SparkMax(BaseConfig.kSpark.MOTOR_ID);
        SparkMaxConfig shooterConfig = new SparkMaxConfig();

         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor1.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        shooterMotor1.setCANTimeout(500);//Units in miliseconds
  
    //shooterMotor2 = new SparkMax(BaseConfig.ShooterConstants.MOTOR_ID);

         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor1.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        shooterMotor2.setCANTimeout(500);//Units in miliseconds

    //feederMotor = new SparkMax(BaseConfig.ShooterConstants.MOTOR_ID);
    
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor1.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        feederMotor.setCANTimeout(500);//Units in miliseconds


    //indexerMotor = new SparkMax(BaseConfig.ShooterConstants.MOTOR_ID);
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        indexerMotor.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        indexerMotor.setCANTimeout(500);//Units in miliseconds
  }
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