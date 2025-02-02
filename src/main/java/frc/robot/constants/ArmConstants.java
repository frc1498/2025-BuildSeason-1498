package frc.robot.constants;

public class ArmConstants {

    //===============Coral Positions======================
    public static final double kCoralStow = 0.0;  //Stow Position 
    public static final double kCoralLoadFloor = 0.1;  //  Load Coral Floor Position
    public static final double kCoralLoadHuman = 0.1;  //  Load Coral Floor Position 
    public static final double kCoralL1 = 0.3; // Level 1 Score Coral Position
    public static final double kCoralL2 = 0.4; // Level 2 Score Coral Position
    public static final double kCoralL3 = 0.5; // Level 3 Score Coral Position
    public static final double kCoralL4 = 0.6; // Level 4 Score Coral Position

    //==================Algae Positions====================
    public static final double kAlgaeStow = 0.1; // Load Algae Floor Position
    public static final double kAlgaeLoadFloor = 0.1; // Load Algae Floor Position
    public static final double kAlgaeLoadL2= 0.1; // Load Algae L2 Position
    public static final double kAlgaeLoadL3 = 0.1; // Load Algae L3 Position 
    public static final double kAlgaeBarge = 0.9; //Score position for Algae in the Barge
    public static final double kAlgaeProcessor = 0.1; //Score position for Algae in the processor

    //=======================General======================
    public static final double kDeadband = 0.05;

//==================Sim Values================
public static final double kArmRotateGearing = 126.0; //126:1 reduction
public static final double kArmRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea.
public static final double kArmLength = 0.381; //meters - Guess that the arm is 15 in. long.
public static final double kArmMinAngle = 0.0; //radians
public static final double kArmMaxAngle = 6.28; //2*pi in radians - 360 degrees of movement.
public static final double kArmStartingAngle = 0.0; //radians

}