package frc.robot.constants;

public class WristConstants {

    //===============Coral Positions======================
    public static final double kCoralStow = 0.0;  //Stow Position - 0 degrees
    public static final double kCoralLoadFloor = 0.0;  //  Load Coral Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kCoralLoadHuman = 0.0;  //  Load Coral Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kCoralL1 = -3.127; // Level 1 Score Coral Position - -15 degrees - -0.262 radians - -3.127 rotations
    public static final double kCoralL2 = -4.166; // Level 2 Score Coral Position - -20 degrees - -0.349 radians - -4.166 rotations
    public static final double kCoralL3 = -5.204; // Level 3 Score Coral Position - -25 degrees - -0.436 radians - -5.204 rotations
    public static final double kCoralL4 = -6.255; // Level 4 Score Coral Position - 30 degrees - -0.524 radians - -6.255 rotations

    //==================Algae Positions====================
    public static final double kAlgaeStow = 0; // Load Algae Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kAlgaeLoadFloor = 0; // Load Algae Floor Position - 0 degrees - 0 radians - 0 rotations
    public static final double kAlgaeL2= 4.166; // Load Algae L2 Position - 20 degrees - 0.349 radians - 4.166 rotations
    public static final double kAlgaeL3 = 5.204; // Load Algae L3 Position - 25 degrees - 0.436 radians - 5.204 rotations
    public static final double kAlgaeBarge = 9.370; //Score position for Algae in the Barge - 45 degrees - 0.785 radians - 9.370 rotations
    public static final double kAlgaeProcessor = 0; //Score position for Algae in the processor - 0 degrees - 0 radians - 0 rotations
    //==================General Positions====================
    public static final double kFrontSafe = 0;
    public static final double kRearSafe = 0;
    
    //===============Wrist Intake Spin Velocities======================
    public static final double kCoralStop = 0.0;  //Stow Position 
    public static final double kCoralSuck = 50.0;  //  Load Coral Floor Position
    public static final double kCoralSpit = -50.0;  //  Load Coral Floor Position 
    public final static double kCoralSlowBackward = -25; //Move coral slowly in gripper forward
    public final static double kCoralSlowForward = 50; //Move coral slowly in gripper backward


    //=======================General======================
    public static final double kDeadband = 0.05;

    //==================Sim Values================
    public static final double kWristRotateGearing = 75.0; //75:1 reduction
    public static final double kWristRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea
    public static final double kWristLength = 0.1905; //meters - Assume wrist is 15 in. long, pivot is in the center, so actually 7.5 in.
    public static final double kWristMinAngle = -100; //radians
    public static final double kWristMaxAngle = 100; //2*pi in radians - 360 degrees rotation
    public static final double kWristStartingAngle = 0.0; //radians
    public static final double kWristRollerMomentOfInertia = 0.01; //kg-m^2
    public static final double kWristRollerGearing = 3.0; //3:1 reduction
}
