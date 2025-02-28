package frc.robot.constants;

public class WristConstants {

    //===============Coral Positions======================
    public static final double kCoralStow = 0.0;  //Stow Position - 0 degrees
    public static final double kCoralLoadFloor = -0.18;  //  Load Coral Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kCoralLoadHuman = -0.0;  //  Load Coral Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kCoralL1 = -0.08; // Level 1 Score Coral Position - Changing to real measurements, was:-15 degrees - -0.262 radians - -3.127 rotations
    public static final double kCoralL2 = 0.09; // Level 2 Score Coral Position - Changing to real measurements, was:-20 degrees - -0.349 radians - -4.166 rotations
    public static final double kCoralL3 = -0.44; // Level 3 Score Coral Position - Changing to real measurements, was:-25 degrees - -0.436 radians - -5.204 rotations
    public static final double kCoralL4 =-0.44; // Level 4 Score Coral Position - Changing to real measurements, was: 30 degrees - -0.524 radians - -6.255 rotations

    public static final double kCoralLoadFloorBetterInitial = -0.3;
    public static final double kCoralLoadFloorBetterFinal = -0.15;
    

    //==================Algae Positions====================
    public static final double kAlgaeStow = 0; // Load Algae Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kAlgaeLoadFloor = 0; // Load Algae Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kAlgaeL2= 0; // Load Algae L2 Position - Changing to real measurements, was: 20 degrees - 0.349 radians - 4.166 rotations
    public static final double kAlgaeL3 = 0; // Load Algae L3 Position - Changing to real measurements, was: 25 degrees - 0.436 radians - 5.204 rotations
    public static final double kAlgaeBarge = 0; //Score position for Algae in the Barge - Changing to real measurements, was: 45 degrees - 0.785 radians - 9.370 rotations
    public static final double kAlgaeProcessor = 0; //Score position for Algae in the processor - Changing to real measurements, was: 0 degrees - 0 radians - 0 rotations
    //==================General Positions====================
    public static final double kFrontSafe = 0;
    public static final double kRearSafe = 0;
    
    //===============Wrist Intake Spin Velocities======================
    public static final double kCoralStop = 0.0;  //Stow Position 
    public static final double kCoralSuck = 9.0;  //  Load Coral Floor Position
    public static final double kCoralL1Spit = 15.0;  //  Load Coral Floor Position 
    public static final double kCoralL2Spit = 30.0;  //  Load Coral Floor Position 
    public static final double kCoralL3Spit = 30.0;  //  Load Coral Floor Position 
    public static final double kCoralL4Spit = 30.0;  //  Load Coral Floor Position 
    public final static double kCoralSlowBackward = -10; //Move coral slowly in gripper forward
    public final static double kCoralSlowForward = 10; //Move coral slowly in gripper backward
    public final static double kCoralClear = -15;

    //Safeties - the allowable range of movement.
    public static final double kWristMinPosition = -0.572266;
    public static final double kWristMaxPosition = 0.181641;

    //=======================General======================
    public static final double kDeadband = 0.015;

    //==================Sim Values================
    public static final double kWristRotateGearing = 75.0; //75:1 reduction
    public static final double kWristRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea
    public static final double kWristLength = 0.1905; //meters - Assume wrist is 15 in. long, pivot is in the center, so actually 7.5 in.
    public static final double kWristMinAngle = -100; //radians
    public static final double kWristMaxAngle = 100; //2*pi in radians - 360 degrees rotation
    public static final double kWristStartingAngle = 0.0; //radians
    public static final double kWristRollerMomentOfInertia = 0.01; //kg-m^2
    public static final double kWristRollerGearing = 3.0; //3:1 reduction

    //=================Can Range Values===============
    public static final double krangeL2 = 0;
    public static final double krangeL3 = 0;
    public static final double krangeL4 = 0;
    
}
