package frc.robot.constants;

public class ElevatorConstants {
    

    //Numbers are for simulation for now.
    //Positions are in meters.  Heights are based off the manual, so not quite practical for actual scoring.
    public static final double kStow = 0;
    
    public static final double kCoralMax = 32.0; //Rotations
    public static final double kCoralMin = 0.5; //Rotations

    //===========Coral Positions===================
    public static final double kCoralStow = 5; //Keep this the same as KCoral Load Floor to save motion
    public static final double kCoralLoadFloor = 5; //was 5, changing to 2 for testing
    public static final double kCoralLoadHuman = 17.5; //0.1 meters
    public static final double kCoralL1 = 7; //0.46 meters
    public static final double kCoralL2 = 29; //0.81 meters
    public static final double kCoralL3 = 0.5; //1.21 meters
    public static final double kCoralL4 = 27; //1.83 meters

    //=============Algae Positions=============
    public static final double kAlgaeStow = 0.0;
    public static final double kAlgaeLoadFloor = 4; //0.1 meters  
    public static final double kAlgaeL2 = 9; //0.81 meters
    public static final double kAlgaeL3 = 24; //1.21 meters
    public static final double kAlgaeBarge = 19; //2.2 meters
    public static final double kAlgaeProcessor = 24; //0.3 meters

    //==================General===================
    //Deadband to determine if the elevator is 'at' a position.
    public static final double kDeadband = 0.15;
    public static final double kFrontSafe = 25;
    public static final double kRearSafe = 28;  //Keep this the same as L4 to speed that motion
    
    //==================Sim Values================
    public static final double kElevatorGearing = 7.75;  //Assumes the form of X:1, so anything greater than 1 is a reduction.
    public static final double kElevatorCarriageMass = 6.80; //kg - Estimating the arm and wrist together weigh ~15 lb., so ~ 6.8 kg.
    public static final double kElevatorDrumRadius = 0.2159; //m - The 'drum' is linear, and it is 17 in tall, so estimate an 8.5 in. 'radius'
    public static final double kElevatorMinHeight = 0.0; //m
    public static final double kElevatorMaxHeight = 5.0; //m - Assume each stage is 17 in., and there are 3 stages.
    public static final double kElevatorStartingHeight = 0.0; //m
}
