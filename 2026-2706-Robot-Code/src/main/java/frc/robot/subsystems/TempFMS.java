package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempFMS extends SubsystemBase{
    private final PhotonSubsystem m_PhotonSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;

    private DoublePublisher shooterRPMPublisher;
    private DoublePublisher distanceToAprilTagPublisher;

    private IntegerSubscriber customRPMSubscriber;

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
        customRPMSubscriber = networkTable.getIntegerTopic("Variable RPM").subscribe(0);
    }

    @Override
    public void periodic() {
        rpm = m_ShooterSubsystem.getShooterRPM();
        shooterRPMPublisher.set(rpm);

        distance = m_PhotonSubsystem.getDistance();
        distanceToAprilTagPublisher.set(distance);

        m_ShooterSubsystem.customRPM = (int) customRPMSubscriber.get();
    }


}
