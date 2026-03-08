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
  private final SparkClosedLoopController m_pidController; // New

  public ShooterSubsystem() {
    shooterMotor1 = new SparkMax(Constants.shooterConstants.MOTOR1_ID, MotorType.kBrushless);
    shooterMotor2 = new SparkMax(Constants.shooterConstants.MOTOR2_ID, MotorType.kBrushless);
    feederMotor = new SparkMax(Constants.shooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
    indexerMotor = new SparkMax(Constants.shooterConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);

    m_encoder = shooterMotor1.getEncoder();
    m_pidController = shooterMotor1.getClosedLoopController();

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.inverted(false);
    
    // Add PIDF Gains here
    shooterConfig.closedLoop.p(0);         
    shooterConfig.closedLoop.i(0);
    shooterConfig.closedLoop.d(0);
    shooterConfig.closedLoop.velocityFF(0.0);
    shooterConfig.closedLoop.outputRange(-1, 1);

    shooterMotor1.configure(shooterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(shooterMotor1); // Tells motor 2 to do whatever motor 1 does
    shooterMotor2.configure(followerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);
    feederMotor.configure(feederConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    feederConfig.closedLoop.p(0);         
    feederConfig.closedLoop.i(0);
    feederConfig.closedLoop.d(0);
    feederConfig.closedLoop.velocityFF(0);
    feederConfig.closedLoop.outputRange(-1, 1);
    
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.inverted(false);
    indexerMotor.configure(indexerConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    indexerConfig.closedLoop.p(0);         
    indexerConfig.closedLoop.i(0);
    indexerConfig.closedLoop.d(0);
    indexerConfig.closedLoop.velocityFF(0.15);
    indexerConfig.closedLoop.outputRange(-1, 1);
  }

  public boolean isRPMinRange() {
    double currentRPM = m_encoder.getVelocity();
    double tolerance = 200;
    return Math.abs(currentRPM - getDesiredVelocityRPM()) < tolerance;
  }

  public double getDesiredVelocityRPM() {
    return 2000; 
  }

  public void stop() {
    shooterMotor1.stopMotor(); 
    feederMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  public void spinningUp() {
    m_pidController.setReference(getDesiredVelocityRPM(), SparkBase.ControlType.kVelocity);
    feederMotor.stopMotor();
    indexerMotor.stopMotor();
  }

  public void ready() {
    m_pidController.setReference(getDesiredVelocityRPM(), SparkBase.ControlType.kVelocity);
    feederMotor.set(0.5); 
    indexerMotor.set(0.5); 
  }
}