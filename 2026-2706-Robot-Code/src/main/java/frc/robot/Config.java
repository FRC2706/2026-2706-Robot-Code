package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class Config {

    private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");

    private static int robotId = -1;

    public static class CANID {
    public static final int CANDLE = robotSpecific(25,-1,15,25);
    }

    private static final int SIMULATION_ID = 1;
    
    @SafeVarargs
    public static <T> T robotSpecific(T first, T... more) {
    try {
        return more[getRobotId()-1];
    } catch (Exception e) {
        return first;
    }
    }


    public static int getRobotId() {

    if (robotId < 0) {
        // Backup in case the FMS is attached, force to comp robot
        if (DriverStation.isFMSAttached()) {
        robotId = 0;
        }

        // Set the Id to the simulation if simulating
        else if (RobotBase.isSimulation()) {
        robotId = SIMULATION_ID;

        // Not simulation, read the file on the roborio for it's robot id.
        } else {
        try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
            robotId = Integer.parseInt(reader.readLine());
        } catch (Exception e) {
            robotId = 0; // DEFAULT TO COMP ROBOT IF NO ID IS FOUND
        }
        }
    }

    return robotId;
    }
}
