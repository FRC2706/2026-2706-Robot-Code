package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController; // New
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;
  private final SparkMax feederMotor;
  private final SparkMax indexerMotor;
  
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pidControllerShooter; // New
  private final SparkClosedLoopController m_pidControllerFeeder; // New
  private final SparkClosedLoopController m_pidControllerIndexer; // New



  public ShooterSubsystem() {
    shooterMotor1 = new SparkMax(Constants.shooterConstants.MOTOR1_ID, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(Constants.shooterConstants.MOTOR2_ID, MotorType.kBrushless);
    feederMotor = new SparkMax(Constants.shooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    indexerMotor = new SparkMax(Constants.shooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    m_encoder = shooterMotor1.getEncoder();
    m_pidControllerShooter = shooterMotor1.getClosedLoopController();
    m_pidControllerFeeder = feederMotor.getClosedLoopController();
    m_pidControllerIndexer = indexerMotor.getClosedLoopController();

    int currentLimit = 40;

    //-------Shooter Motors configuration & PID-----------//

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.inverted(false);
    
    shooterConfig.closedLoop.p(0.00005);         
    shooterConfig.closedLoop.i(0);
    shooterConfig.closedLoop.d(0);
    shooterConfig.closedLoop.velocityFF(0.0021);
    shooterConfig.closedLoop.outputRange(-1, 1);
   
    shooterConfig.smartCurrentLimit(currentLimit);

    shooterMotor1.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(shooterMotor1); // Tells motor 2 to do whatever motor 1 does
    shooterMotor2.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    //-------Feeder & Indexer configuration & PID-----------//

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);

    feederConfig.closedLoop.p(0.000001); // will change later    
    feederConfig.closedLoop.i(0);
    feederConfig.closedLoop.d(0);
    feederConfig.closedLoop.velocityFF(0.00015);
    feederConfig.closedLoop.outputRange(-1, 1);
    feederConfig.smartCurrentLimit(currentLimit);


    feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.inverted(false);

    indexerConfig.closedLoop.p(0.000001);  // will change later   
    indexerConfig.closedLoop.i(0);
    indexerConfig.closedLoop.d(0);
    indexerConfig.closedLoop.velocityFF(0.00015);
    indexerConfig.closedLoop.outputRange(-1, 1);
    indexerConfig.smartCurrentLimit(currentLimit);

    indexerMotor.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    //-------------------------//
  }

  public boolean isRPMinRange() {
    double currentRPM = m_encoder.getVelocity();
    double tolerance = 200;
    System.out.println(currentRPM);

    return (Math.abs(currentRPM - getDesiredVelocityRPM()) < tolerance);
  }

  public double getDesiredVelocityRPM() {
    return 2300; 
  }

  public void stop() {
    shooterMotor1.stopMotor(); 
    feederMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  public void spinningUp() {
    m_pidControllerShooter.setReference(getDesiredVelocityRPM(), SparkBase.ControlType.kVelocity);
    feederMotor.stopMotor();
    indexerMotor.stopMotor();
    System.out.println("spinning up");

  }

  public void ready() {
    m_pidControllerShooter.setReference(getDesiredVelocityRPM(), SparkBase.ControlType.kVelocity);
    
    // figure out how much faster this shoudl go
    m_pidControllerFeeder.setReference(getDesiredVelocityRPM()*1.2, SparkBase.ControlType.kVelocity);
    m_pidControllerIndexer.setReference(getDesiredVelocityRPM()*1.2, SparkBase.ControlType.kVelocity);
    //feederMotor.setReference(0.5); 
    //indexerMotor.set(0.5); 
    System.out.println("ready");

  }
}

// 35-40 amps for indexer
// 