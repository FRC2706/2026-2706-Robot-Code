package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController; // New
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityConstants;
import frc.robot.UtilityConstants.shooterConstants;

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
    shooterMotor1 = new SparkMax(UtilityConstants.shooterConstants.MOTOR1_ID, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(UtilityConstants.shooterConstants.MOTOR2_ID, MotorType.kBrushless);
    feederMotor = new SparkMax(UtilityConstants.shooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    indexerMotor = new SparkMax(UtilityConstants.shooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    m_encoder = shooterMotor1.getEncoder();
    m_pidControllerShooter = shooterMotor1.getClosedLoopController();
    m_pidControllerFeeder = feederMotor.getClosedLoopController();
    m_pidControllerIndexer = indexerMotor.getClosedLoopController();

    int currentLimit = 35;

    //-------Shooter Motors configuration & PID-----------//

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.inverted(false);
    
    shooterConfig.closedLoop.p(shooterConstants.shooterkP);         
    shooterConfig.closedLoop.i(shooterConstants.shooterkI);
    shooterConfig.closedLoop.d(shooterConstants.shooterkD);
    shooterConfig.closedLoop.velocityFF(shooterConstants.shooterkFF);
    shooterConfig.closedLoop.outputRange(-1, 1);
   
    shooterConfig.smartCurrentLimit(currentLimit);

    shooterMotor1.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(shooterMotor1); // Tells motor 2 to do whatever motor 1 does
    shooterMotor2.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


    //-------Feeder & Indexer configuration & PID-----------//

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);

    feederConfig.closedLoop.p(shooterConstants.feederkP); // will change later    
    feederConfig.closedLoop.i(shooterConstants.feederkI);
    feederConfig.closedLoop.d(shooterConstants.feederkD);
    feederConfig.closedLoop.velocityFF(shooterConstants.feederkFF);
    feederConfig.closedLoop.outputRange(-1, 1);
    feederConfig.smartCurrentLimit(currentLimit);


    feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.inverted(true);

    indexerConfig.closedLoop.p(shooterConstants.indexerkP);  // will change later   
    indexerConfig.closedLoop.i(shooterConstants.indexerkI);
    indexerConfig.closedLoop.d(shooterConstants.indexerkD);
    indexerConfig.closedLoop.velocityFF(shooterConstants.indexerkFF);
    indexerConfig.closedLoop.outputRange(-1, 1);
    indexerConfig.smartCurrentLimit(currentLimit);

    indexerMotor.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    //-------------------------//
  }

  public boolean isRPMinRange(int position) {
    double currentRPM = m_encoder.getVelocity();
    double tolerance = 150;
    System.out.println(currentRPM);

    return (Math.abs(currentRPM - getDesiredVelocityRPM(position)) < tolerance);
  }

 
  public int getDesiredVelocityRPM(int position) {
    switch (position) {
      case 0: // hub
        return 1950;
      case 1: // trench
        return 3250;
      case 2: // back wall
        return 3700;
      default:
        return 2650; // this is a fallback RPM, avg of other RPMs
    }
  }

  public void stop() {
    shooterMotor1.stopMotor(); 
    feederMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  public void spinningUp(int position) {
    m_pidControllerShooter.setSetpoint(getDesiredVelocityRPM(position), SparkBase.ControlType.kVelocity);
    feederMotor.stopMotor();
    indexerMotor.stopMotor();

  }

  public void ready(int position) {
    m_pidControllerShooter.setSetpoint(getDesiredVelocityRPM(position), SparkBase.ControlType.kVelocity);
    
    // figure out how much faster this shoudl go
    m_pidControllerFeeder.setSetpoint(getDesiredVelocityRPM(position)*11, SparkBase.ControlType.kVelocity);
    m_pidControllerIndexer.setSetpoint(getDesiredVelocityRPM(position)*12, SparkBase.ControlType.kVelocity);
    //feederMotor.setReference(0.5); 
    //indexerMotor.set(0.5); 

  }
}
