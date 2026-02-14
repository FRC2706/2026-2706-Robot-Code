package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    
    @SuppressWarnings("resource")
        SparkMax shooterMotor1 = new SparkMax(Constants.shooterConstants.MOTOR1_ID, MotorType.kBrushless);
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor1.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters);
        shooterMotor1.setCANTimeout(500);//Units in miliseconds



    @SuppressWarnings("resource")
        SparkMax shooterMotor2 = new SparkMax(Constants.shooterConstants.MOTOR2_ID, MotorType.kBrushless);
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor2.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        shooterMotor2.setCANTimeout(500);//Units in miliseconds


        
    @SuppressWarnings("resource")
        SparkMax feederMotor = new SparkMax(Constants.shooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        feederMotor.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        feederMotor.setCANTimeout(500);//Units in miliseconds



    @SuppressWarnings("resource")
        SparkMax indexerMotor = new SparkMax(Constants.shooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
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