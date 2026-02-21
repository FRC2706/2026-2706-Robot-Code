package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;



public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor1;
  private SparkMax shooterMotor2;
  private SparkMax feederMotor; // CANNOT be faster than shooterMotor RPM
  private SparkMax indexerMotor;
  private RelativeEncoder m_encoder;
  
  public ShooterSubsystem() {
        shooterMotor1 = new SparkMax(Constants.shooterConstants.MOTOR1_ID, MotorType.kBrushless);
        m_encoder = shooterMotor1.getEncoder();
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor1.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        shooterMotor1.setCANTimeout(500);//Units in miliseconds



    //@SuppressWarnings("resource")
        shooterMotor2 = new SparkMax(Constants.shooterConstants.MOTOR2_ID, MotorType.kBrushless);
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        shooterMotor2.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        shooterMotor2.setCANTimeout(500);//Units in miliseconds
                SparkMaxConfig followerConfig =  new SparkMaxConfig();
                 followerConfig.follow(shooterMotor1);
                 shooterMotor2.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
                 




        
    //@SuppressWarnings("resource")
        feederMotor = new SparkMax(Constants.shooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
         // Determines which way the motor spins
        shooterConfig.inverted(false);
        feederMotor.configure( 
        shooterConfig,
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
        );
        feederMotor.setCANTimeout(500);//Units in miliseconds



    // @SuppressWarnings("resource")
    //     SparkMax indexerMotor = new SparkMax(Constants.shooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    //      // Determines which way the motor spins
    //     shooterConfig.inverted(false);
    //     indexerMotor.configure( 
    //     shooterConfig,
    //     SparkBase.ResetMode.kResetSafeParameters, 
    //     SparkBase.PersistMode.kPersistParameters
    //     );
    //     indexerMotor.setCANTimeout(500);//Units in miliseconds

    //-----------------------------------------------


  }

  public void testMotor() { // purely for testing
    System.out.println(shooterMotor1);
    System.out.println(shooterMotor2);
    System.out.println(feederMotor);

    shooterMotor1.set(getDesiredVoltage());
    System.out.println("motor 1 works");
    shooterMotor2.set(getDesiredVoltage());
    System.out.println("motor 2 works");
    feederMotor.set(getDesiredVoltage());
    System.out.println("motor feeder works");
  }

  /**
  
   * @return 
   */
  
  public Boolean isRPMinRange() {
    double currentRPM = m_encoder.getVelocity();

    double rangeLowEnd = getDesiredVelocityRPM() - 100;
    double rangeHighEnd = getDesiredVelocityRPM() + 100;
    System.out.println(currentRPM);
    if (rangeLowEnd < currentRPM && currentRPM < rangeHighEnd) {
      return true;
    } else {
      return false;
    }
  }



  public double getDesiredVoltage(){
    return 0.17; // for testing purposes
    //return getDesiredVoltage();
     // Use calculated RPM to set voltage used by motors
  }

  public double getDesiredVelocityRPM (){
    return 1000; // for test purposes
    
    //return getDesiredVelocityRPM (); 
    //Goal: Get average shooting distance from hardware and set an average RPM
    //Reach goal: Use data provided by vision snensors (distance from hub) to calculate speed needed
  }



  public void stop(){
        System.out.println("stop cmd called");
        shooterMotor1.stopMotor(); 
        feederMotor.stopMotor();
        //indexerMotor.stopMotor();
  }

  public void spinningUp(){
        System.out.println("spining up cmd called");
        //shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        //shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor1.set(getDesiredVoltage());
        feederMotor.stopMotor();
        //indexerMotor.stopMotor();
  }

  public void ready(){
        System.out.println("ready cmd called");
        //shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        //shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor1.set(getDesiredVoltage());
        // divide by 2 is placeholder but need to figure out difference in voltage for feeder+indexer vs shooter motors
        
        //feederMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2); 
        //indexerMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2);

        feederMotor.set(getDesiredVoltage()/2); 
        //indexerMotor.set(getDesiredVoltage()/2); 
  }

  //@Override
  // public void periodic() {
  //   System.out.println("in start shooter if condition");
  //   if (isRPMinRange()) {
  //     ready();
  //   } else {
  //     spinningUp();
  //   }
  
  // }

  @Override
  public void simulationPeriodic() {

  }
}