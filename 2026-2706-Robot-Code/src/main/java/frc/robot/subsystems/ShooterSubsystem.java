package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterMotor1;
  private SparkMax shooterMotor2;
  private SparkMax feederMotor; // CANNOT be faster than shooterMotor RPM
  private SparkMax indexerMotor;
  private RelativeEncoder m_encoder = shooterMotor1.getEncoder();

  // private static ShooterSubsystem shooter;
  //   public static ShooterSubsystem getInstance() {
  //       if (shooter == null)
  //           shooter = new ShooterSubsystem();
  //       return shooter;
  //   }
  
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

    //-----------------------------------------------
  }
  /**
  
   * @return 
   */
  /** 
  //Two different modes we are using
    private ShooterModes desiredMode = ShooterModes.STOP_SHOOTER;
    private States currentState = States.IN_IDLE;

    //Defining different modes and their parameters(RPM and Voltage)
    public static enum ShooterModes {
        STOP_SHOOTER(0, 0),
        SHOOT(0,0);

        double v, RPM;

        private ShooterModes (double voltage, double velo){
            v = voltage;
            RPM = velo;
        }

        public double getDesiredVoltage(){
            return v;
        }

        public double getDesiredSpeedRPM () {
            return RPM;
        }
  } */

  public Boolean isRPMinRange() {
    double currentRPM = m_encoder.getVelocity();
    double rangeLowEnd = getDesiredVelocityRPM() - 100;
    double rangeHighEnd = getDesiredVelocityRPM() + 100;
    if (rangeLowEnd < currentRPM && currentRPM < rangeHighEnd) {
      return true;
    } else {
      return false;
    }
  }

  /**
  //Defining states we are using
  public static enum States {
    IN_IDLE, // shooter, feeder, indexer off
    SPINNING_UP, // shooter spinning to desired rpm, feeder, indexer off
    READY // everything on
  }
  //Setting desired mode and parameters
  public void setMode (ShooterModes desiredMode){
    this.desiredMode = desiredMode;

  } */

  public double getDesiredVoltage(){
    return 0.5; // for testing purposes
    //return getDesiredVoltage();
     // Use calculated RPM to set voltage used by motors
  }

  public double getDesiredVelocityRPM (){
    return 1000; // for test purposes
    
    //return getDesiredVelocityRPM (); 
    //Goal: Get average shooting distance from hardware and set an average RPM
    //Reach goal: Use data provided by vision snensors (distance from hub) to calculate speed needed
  }

  /** Logging the needed values
  public ShooterModes getDesiredMode(){
        return desiredMode;
  }
  //Returns current state
  public States getCurrentState(){
    return currentState;
  } */


  public void stop(){
        System.out.println("stop cmd called");
        shooterMotor1.stopMotor(); 
        shooterMotor2.stopMotor(); 
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
  }

  public void spinningUp(){
        System.out.println("spining up cmd called");
        //shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        //shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor1.set(getDesiredVoltage());
        shooterMotor2.set(getDesiredVoltage());
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
  }

  public void ready(){
        System.out.println("ready cmd called");
        //shooterMotor1.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        //shooterMotor2.set(ShooterModes.SHOOT.getDesiredVoltage()); //set desired voltage
        shooterMotor1.set(getDesiredVoltage());
        shooterMotor2.set(getDesiredVoltage());
        // divide by 2 is placeholder but need to figure out difference in voltage for feeder+indexer vs shooter motors
        
        //feederMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2); 
        //indexerMotor.set(ShooterModes.SHOOT.getDesiredVoltage()/2);

        feederMotor.set(getDesiredVoltage()/2); 
        indexerMotor.set(getDesiredVoltage()/2); 



  }

  public void testMotor() { // purely for testing
    shooterMotor1.set(getDesiredVoltage());
    shooterMotor2.set(getDesiredVoltage());
    feederMotor.set(getDesiredVoltage());
  }



  @Override
  public void periodic() {

    /**
        switch (currentState){
        case IN_IDLE: // is the button pressed down
            stop();
            break;
        case SPINNING_UP: // button is pressed down
            spinningUp();
            break;
        case READY:
            ready();
            break;
        default: 
            break;
    } */
  }

  @Override
  public void simulationPeriodic() {
    testMotor();
  }
}