package frc.robot.subsystems;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilityConstants;

public class TempFMS extends SubsystemBase{

    //TO BE REMOVED

    private final PhotonSubsystem m_PhotonSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;

    private DoublePublisher shooterRPMPublisher;
    private DoublePublisher distanceToAprilTagPublisher;

    private IntegerSubscriber customRPMSubscriber;
    private DoubleSubscriber normalPSubscriber;
    private DoubleSubscriber normalISubscriber;
    private DoubleSubscriber normalDSubscriber;
    private DoubleSubscriber aggressivePSubscriber;
    private DoubleSubscriber aggressiveISubscriber;
    private DoubleSubscriber aggressiveDSubscriber;

    private double normalP;
    private double normalI;
    private double normalD;
    private double aggressiveP;
    private double aggressiveI;
    private double aggressiveD;

    private double currentNormalP = UtilityConstants.shooterConstants.shooterkP;
    private double currentNormalI = UtilityConstants.shooterConstants.shooterkI;
    private double currentNormalD = UtilityConstants.shooterConstants.shooterkD;
    private double currentAggressiveP = UtilityConstants.shooterConstants.shooterAgressivekP;
    private double currentAggressiveI = UtilityConstants.shooterConstants.shooterAgressivekI;
    private double currentAggressiveD = UtilityConstants.shooterConstants.shooterAgressivekD;

    private static double rpm = 0;
    private static double distance = 0;
    
    public TempFMS(PhotonSubsystem photonSubsystem, ShooterSubsystem shooterSubsystem){
        //Subsystems we want data on/send data to
        m_PhotonSubsystem = photonSubsystem;
        m_ShooterSubsystem = shooterSubsystem;

        //Set up network table to have current distance to april tag and rpm and to take in a target rpm
        NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        NetworkTable networkTable = networkTableInstance.getTable("datatable");
        shooterRPMPublisher = networkTable.getDoubleTopic("Shooter RPM").publish();
        distanceToAprilTagPublisher = networkTable.getDoubleTopic("Distance to April Tag").publish();

        //Publish all inputs so they can be seen
        networkTable.getIntegerTopic("Variable RPM").publish();
        networkTable.getDoubleTopic("Normal P").publish();
        networkTable.getDoubleTopic("Normal I").publish();
        networkTable.getDoubleTopic("Normal D").publish();
        networkTable.getDoubleTopic("Aggressive P").publish();
        networkTable.getDoubleTopic("Aggressive I").publish();
        networkTable.getDoubleTopic("Aggressive D").publish();

        //Set up inputs to get input
        customRPMSubscriber = networkTable.getIntegerTopic("Variable RPM").subscribe(1700);
        normalPSubscriber = networkTable.getDoubleTopic("Normal P").subscribe(UtilityConstants.shooterConstants.shooterkP);
        normalISubscriber = networkTable.getDoubleTopic("Normal I").subscribe(UtilityConstants.shooterConstants.shooterkI);
        normalDSubscriber = networkTable.getDoubleTopic("Normal D").subscribe(UtilityConstants.shooterConstants.indexerkD);
        aggressivePSubscriber = networkTable.getDoubleTopic("Aggressive P").subscribe(UtilityConstants.shooterConstants.shooterAgressivekP);
        aggressiveISubscriber = networkTable.getDoubleTopic("Aggressive I").subscribe(UtilityConstants.shooterConstants.shooterAgressivekI);
        aggressiveDSubscriber = networkTable.getDoubleTopic("Aggressive D").subscribe(UtilityConstants.shooterConstants.shooterAgressivekD);
    }

    @Override
    public void periodic() {
        rpm = m_ShooterSubsystem.getShooterRPM();
        shooterRPMPublisher.set(rpm);

        distance = m_PhotonSubsystem.getDistance();
        distanceToAprilTagPublisher.set(distance);

        m_ShooterSubsystem.customRPM = (int) customRPMSubscriber.get();

        normalP = normalPSubscriber.get();
        normalI = normalISubscriber.get();
        normalD = normalDSubscriber.get();
        aggressiveP = aggressivePSubscriber.get();
        aggressiveI = aggressiveISubscriber.get();
        aggressiveD = aggressiveDSubscriber.get();

        if (normalP != currentNormalP || normalI != currentNormalI || normalD != currentNormalD){
            currentNormalP = normalP;
            currentNormalI = normalI;
            currentNormalD = normalD;

            m_ShooterSubsystem.changeNormalPID(currentNormalP, currentNormalI, currentNormalD);
        }

        if (aggressiveP != currentAggressiveP || aggressiveI != currentAggressiveI || aggressiveD != currentAggressiveD){
            currentAggressiveP = aggressiveP;
            currentAggressiveI = aggressiveI;
            currentAggressiveD = aggressiveD;

            m_ShooterSubsystem.changeAggressivePID(currentAggressiveP, currentAggressiveI, currentAggressiveD);
        }
    }


}
