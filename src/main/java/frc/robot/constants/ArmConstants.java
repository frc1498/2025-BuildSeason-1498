package frc.robot.constants;

public class ArmConstants {

    //===============Coral Positions======================
    //Numbers are currently for simulation.
    public static final double kCoralStow = 0.0;  //Stow Position - 0 degrees 
    public static final double kCoralLoadFloor = 5.254;  //  Load Coral Floor Position - 15 degrees - 0.262 radians - 5.254 rotations
    public static final double kCoralLoadHuman = 6.999;  //  Load Coral Floor Position - 20 degrees - 0.349 radians - 6.999 rotations
    public static final double kCoralL1 = 10.548; // Level 1 Score Coral Position - 30 degrees - 0.526 radians - 10.548 rotations
    public static final double kCoralL2 = 15.742; // Level 2 Score Coral Position - 45 degrees - 0.785 radians - 15.742 rotations
    public static final double kCoralL3 = 20.996; // Level 3 Score Coral Position - 60 degrees - 1.047 radians - 20.996 rotations
    public static final double kCoralL4 = 26.250; // Level 4 Score Coral Position - 75 degrees - 1.309 radians - 26.250 rotations

    //==================Algae Positions====================
    public static final double kAlgaeStow = -3.509; // Load Algae Floor Position - -10 degrees - -0.175 radians - -3.509 rotations
    public static final double kAlgaeLoadFloor = -5.254; // Load Algae Floor Position - -15 degrees - -0.262 radians - -5.254 rotations
    public static final double kAlgaeLoadL2= -15.742; // Load Algae L2 Position - -45 degrees - -0.785 radians - -15.742 rotations
    public static final double kAlgaeLoadL3 = -20.996; // Load Algae L3 Position - -60 degrees - -1.047 radians - -20.996 rotations
    public static final double kAlgaeBarge = -29.759; //Score position for Algae in the Barge - -85 degrees - -1.484 radians - -29.759 rotations
    public static final double kAlgaeProcessor = -6.999; //Score position for Algae in the processor - -20 degrees - -0.349 radians - -6.999 rotations

    //=======================General======================
    public static final double kDeadband = 2.0;
    public static final double kFrontSafe=0;
    public static final double kRearSafe=0;

//==================Sim Values================
public static final double kArmRotateGearing = 126.0; //126:1 reduction
public static final double kArmRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea.
public static final double kArmLength = 0.381; //meters - Guess that the arm is 15 in. long.
public static final double kArmMinAngle = -100; //radians
public static final double kArmMaxAngle = 100; //2*pi in radians - 360 degrees of movement.
public static final double kArmStartingAngle = 0.0; //radians

}