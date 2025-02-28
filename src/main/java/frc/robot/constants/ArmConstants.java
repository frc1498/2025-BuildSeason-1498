package frc.robot.constants;

public class ArmConstants {

    //===============Coral Positions======================
    //Numbers are currently for simulation.
    public static final double kCoralStow = -0.20;  //Stow Position - 0 degrees 
    public static final double kCoralLoadFloor = -0.125;  //  Load Coral Floor Position - Changing to real measurements, was: 15 degrees - 0.262 radians - 5.254 rotations
    public static final double kCoralLoadHuman = -0.12;  //  was -.12 Load Coral Floor Position - Changing to real measurements, was:20 degrees - 0.349 radians - 6.999 rotations
    public static final double kCoralL1 = -0.205; // Level 1 Score Coral Position - Changing to real measurements, was:30 degrees - 0.526 radians - 10.548 rotations
    public static final double kCoralL2 = -0.225; // Level 2 Score Coral Position - Changing to real measurements, was:45 degrees - 0.785 radians - 15.742 rotations
    public static final double kCoralL3 = 0.225; // Level 3 Score Coral Position - Changing to real measurements, was:60 degrees - 1.047 radians - 20.996 rotations
    public static final double kCoralL4 = 0.25; // Level 4 Score Coral Position - Changing to real measurements, was:75 degrees - 1.309 radians - 26.250 rotations

    public static final double kCoralLoadFloorBetter = -0.10;

    //==================Algae Positions====================
    public static final double kAlgaeStow = 0; // Load Algae Floor Position - Changing to real measurements, was:-10 degrees - -0.175 radians - -3.509 rotations
    public static final double kAlgaeLoadFloor = 0; // Load Algae Floor Position - Changing to real measurements, was:-15 degrees - -0.262 radians - -5.254 rotations
    public static final double kAlgaeL2= -0.3; // Load Algae L2 Position - -45 degrees - Changing to real measurements, was:-0.785 radians - -15.742 rotations
    public static final double kAlgaeL3 = -0.3; // Load Algae L3 Position - -60 degrees - Changing to real measurements, was:-1.047 radians - -20.996 rotations
    public static final double kAlgaeBarge = 0; //Score position for Algae in the Barge - Changing to real measurements, was:-85 degrees - -1.484 radians - -29.759 rotations
    public static final double kAlgaeProcessor = 0; //Score position for Algae in the processor - Changing to real measurements, was:-20 degrees - -0.349 radians - -6.999 rotations

    //=======================General======================
    public static final double kDeadband = 0.01;
    public static final double kFrontSafe = -0.35;
    public static final double kRearSafe = 0;
    public static final double kIntakeSafe = -.17; //When arm is in middle and moving "through" the intake 
    public static final double kClearClimb = -0.27; //When arm is in middle and moving "through" the intake 

    //=========================Safeties===================
    public static final double kMin = -0.333;
    public static final double  kMax = 0.29;

    //==================Sim Values================
    public static final double kArmRotateGearing = 105.883; //105.883:1 reduction
    public static final double kArmRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea.
    public static final double kArmLength = 0.381; //meters - Guess that the arm is 15 in. long.
    public static final double kArmMinAngle = -100; //radians
    public static final double kArmMaxAngle = 100; //2*pi in radians - 360 degrees of movement.
    public static final double kArmStartingAngle = 0.0; //radians

}