package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController; // New
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityConstants;
import frc.robot.UtilityConstants.shooterConstants;
import frc.robot.UtilityConstants.shooterConstants.shooterPositions;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooterMotor1;
  private final SparkMax shooterMotor2;
  private final SparkMax feederMotor;
  private final SparkMax indexerMotor;
  private final PhotonSubsystem m_PhotonSubsystem;
  
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_pidControllerShooter; // New
  private final SparkClosedLoopController m_pidControllerFeeder; // New
  private final SparkClosedLoopController m_pidControllerIndexer; // New

  public ShooterSubsystem(PhotonSubsystem photonSubsystem) {
    m_PhotonSubsystem = photonSubsystem;
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
  shooterConfig.closedLoop.feedForward.kV(shooterConstants.shooterkFF);
    shooterConfig.closedLoop.outputRange(-1, 1);
   
    shooterConfig.smartCurrentLimit(currentLimit);

  shooterMotor1.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig.follow(shooterMotor1); // Tells motor 2 to do whatever motor 1 does
  shooterMotor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //-------Feeder & Indexer configuration & PID-----------//

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);

    feederConfig.closedLoop.p(shooterConstants.feederkP); // will change later    
    feederConfig.closedLoop.i(shooterConstants.feederkI);
    feederConfig.closedLoop.d(shooterConstants.feederkD);
  feederConfig.closedLoop.feedForward.kV(shooterConstants.feederkFF);
    feederConfig.closedLoop.outputRange(-1, 1);
    feederConfig.smartCurrentLimit(currentLimit);


  feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig.inverted(true);

    indexerConfig.closedLoop.p(shooterConstants.indexerkP);  // will change later   
    indexerConfig.closedLoop.i(shooterConstants.indexerkI);
    indexerConfig.closedLoop.d(shooterConstants.indexerkD);
  indexerConfig.closedLoop.feedForward.kV(shooterConstants.indexerkFF);
    indexerConfig.closedLoop.outputRange(-1, 1);
    indexerConfig.smartCurrentLimit(currentLimit);

  indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //-------------------------//
  }

  // Acceptable yaw error (degrees) to consider the target "aligned" for feeding.
  private static final double kVisionYawDeadbandDeg = 2.0;

  public boolean isRPMinRange(int position) {
    double currentRPM = m_encoder.getVelocity();
    double tolerance = 100;

    return (Math.abs(currentRPM - getDesiredVelocityRPM(position)) < tolerance);
  }

  public int getDesiredVelocityRPM(int position) {
    // Use preset RPMs for known positions, but if position == 3 use vision-distance-based RPM.
    switch (position) {
      case shooterPositions.HUB:
        return 2050;
      case shooterPositions.TRENCH_FAR:
        return 3250;
      case shooterPositions.DEPOT:
        return 3700;
      case shooterPositions.TRENCH_CLOSE:
        return 3150;
      case shooterPositions.OUTPOST:
        return 4030;
      case 3: // Vision-based variable shooting
      {
        double distanceMeters = 0.0;
        if (m_PhotonSubsystem != null) {
          distanceMeters = m_PhotonSubsystem.getDistance();
        }
        // Regression-derived formula (from prior tuning) mapping distance -> RPM.
        int calculatedRPM = Math.min((int) Math.round(13.406342 * distanceMeters + 1878.6562), 5000);
        if (distanceMeters <= 0.0) {
          return 2650; // fallback RPM
        }
        return calculatedRPM;
      }
      default:
        return 2650; // fallback RPM
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
    // Always spin shooter to the vision-calculated RPM.
    m_pidControllerShooter.setSetpoint(getDesiredVelocityRPM(position), SparkBase.ControlType.kVelocity);

  // Only enable feeder/indexer when we have a target, the shooter is up to speed,
  // and the vision yaw is within a small deadband (i.e., robot is aligned to the target).
  if (m_PhotonSubsystem != null && m_PhotonSubsystem.hasTarget() && isRPMinRange(position)
    && Math.abs(m_PhotonSubsystem.getYaw()) <= kVisionYawDeadbandDeg) {
      // Small multipliers to run feeder and indexer slightly faster than the shooter RPM
      m_pidControllerFeeder.setSetpoint(getDesiredVelocityRPM(position) * 1.10, SparkBase.ControlType.kVelocity);
      m_pidControllerIndexer.setSetpoint(getDesiredVelocityRPM(position) * 1.20, SparkBase.ControlType.kVelocity);
    } else {
      // Keep feeder/indexer stopped until aligned and at speed
      feederMotor.stopMotor();
      indexerMotor.stopMotor();
    }
  }

  public void clearIndexer(){
    m_pidControllerIndexer.setSetpoint(4000, SparkBase.ControlType.kVelocity);
  }

  public void stopIndexer(){
    indexerMotor.stopMotor();
  }
}