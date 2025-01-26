package frc.robot.constants;

public class ElevatorConstants {
    public static final int kElevatorDriveFrontCANID = 11;
    public static final int kElevatorDriveBackCANID = 12;

    //Numbers are for simulation for now.
    //Positions are in meters.  Heights are based off the manual, so not quite practical for actual scoring.
    public static final double kHome = 0;
    public static final double kL1Position = 0.46;
    public static final double kL2Position = 0.81;
    public static final double kL3Position = 1.21;
    public static final double kL4Position = 1.83;

    //Deadband to determine if the elevator is 'at' a position.
    //Currently, in meters.
    public static final double kDeadband = 0.05;

    public static final double kElevatorGearing = 1.0;
    public static final double kElevatorCarriageMass = 1.0; //kg
    public static final double kElevatorDrumRadius = 1.0; //m
    public static final double kElevatorMinHeight = 0.0; //m
    public static final double kElevatorMaxHeight = 3.0; //m
    public static final double kElevatorStartingHeight = 0.0; //m
}
