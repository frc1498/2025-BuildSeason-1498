package frc.robot.constants;

public class ElevatorConstants {
    public static final int kElevatorDriveFrontCANID = 11;
    public static final int kElevatorDriveBackCANID = 12;

    //Numbers are for simulation for now.
    //Positions are in meters.  Heights are based off the manual, so not quite practical for actual scoring.
    public static final double kStow = 0;
    
    //===========Coral Positions===================
    public static final double kCoralStow = 0;
    public static final double kCoralLoadFloor = 0.1;
    public static final double kCoralLoadHuman = 0.1;
    public static final double kCoralL1 = 0.46;
    public static final double kCoralL2 = 0.81;
    public static final double kCoralL3 = 1.21;
    public static final double kCoralL4 = 1.83;

    //=============Algae Positions=============
    public static final double kAlgaeStow = 0.0;
    public static final double kAlgaeLoadFloor = 0.1;  
    public static final double kAlgaeL2 = 0.4;
    public static final double kAlgaeL3 = 0.5;
    public static final double kAlgaeBarge = 0.9;
    public static final double kAlgaeProcessor = 0.9;

    //==================General===================
    //Deadband to determine if the elevator is 'at' a position.
    public static final double kDeadband = 0.05;
    //Currently, in meters.
    
    //==================Sim Values================
    public static final double kElevatorGearing = 1.0;
    public static final double kElevatorCarriageMass = 1.0; //kg
    public static final double kElevatorDrumRadius = 1.0; //m
    public static final double kElevatorMinHeight = 0.0; //m
    public static final double kElevatorMaxHeight = 3.0; //m
    public static final double kElevatorStartingHeight = 0.0; //m
}
