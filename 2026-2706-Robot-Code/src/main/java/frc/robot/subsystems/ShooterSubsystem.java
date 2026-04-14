package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController; // New
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityConstants;
import frc.robot.UtilityConstants.shooterConstants;
import frc.robot.UtilityConstants.shooterConstants.shooterPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

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

  public boolean isRPMinRange(int position) {
    double currentRPM = m_encoder.getVelocity();
    double tolerance = 50;

    return (Math.abs(currentRPM - getDesiredVelocityRPM(position)) < tolerance);
  }

  public double getDesiredVelocityRPM(int position) {
    switch (position) {
      case shooterPositions.HUB: 
        return 1700;
      case shooterPositions.TRENCH_FAR: 
        return 2700;
      case shooterPositions.DEPOT:
        return 3700;
      case 3: // variable shooting using the photon distance with the inverse of a quadratic regression formula from an rpm vs. distance graph (soft limit of 5000 RPM)
        //return 0;  
        // Prefer NetworkTables value (allows external processes to override); fallback to PhotonSubsystem if NT missing
        double ntDistance = 0;
        return 1694 + 283*ntDistance + -240*Math.pow(ntDistance, 2) + 82*Math.pow(ntDistance, 3) + -11.4*Math.pow(ntDistance, 4) + 0.688*Math.pow(ntDistance, 5) + -0.0146*Math.pow(ntDistance, 6);
      case shooterPositions.TRENCH_CLOSE: 
        return 3150;
      case shooterPositions.OUTPOST:
        return 4030;
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
    
    // figure out how much faster this should go
    m_pidControllerFeeder.setSetpoint(4000, SparkBase.ControlType.kVelocity);
    m_pidControllerIndexer.setSetpoint(3000, SparkBase.ControlType.kVelocity);
  }

  public void clearIndexer(){
    m_pidControllerIndexer.setSetpoint(4000, SparkBase.ControlType.kVelocity);
  }

  public void stopIndexer(){
    indexerMotor.stopMotor();
  }

  //Feed the shooter at varying speeds depending on the position of the robot; Further positions require lower rpm (lower shooting rate)
  public void feedShooter(int position){
    switch(position){
      case shooterPositions.HUB:
        {
          break;
        }
        
      case shooterPositions.DEPOT:
        {
          break;
        }
    }
  }
}