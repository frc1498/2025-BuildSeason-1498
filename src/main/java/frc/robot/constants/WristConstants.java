package frc.robot.constants;

public class WristConstants {

    //===============Coral Positions======================
    public static final double kCoralStow = 0.0;  //Stow Position 
    public static final double kCoralLoadFloor = 0.1;  //  Load Coral Floor Position
    public static final double kCoralLoadHuman = 0.1;  //  Load Coral Floor Position 
    public static final double kCoralL1 = 50; // Level 1 Score Coral Position
    public static final double kCoralL2 = 0; // Level 2 Score Coral Position
    public static final double kCoralL3 = 0.5; // Level 3 Score Coral Position
    public static final double kCoralL4 = 0.6; // Level 4 Score Coral Position

    //==================Algae Positions====================
    public static final double kAlgaeStow = 0.1; // Load Algae Floor Position
    public static final double kAlgaeLoadFloor = 0.1; // Load Algae Floor Position
    public static final double kAlgaeL2= 0.1; // Load Algae L2 Position
    public static final double kAlgaeL3 = 0.1; // Load Algae L3 Position 
    public static final double kAlgaeBarge = 0.9; //Score position for Algae in the Barge
    public static final double kAlgaeProcessor = 0.1; //Score position for Algae in the processor

    //===============Wrist Intake Spin Velocities======================
    public static final double kCoralStop = 0.0;  //Stow Position 
    public static final double kCoralSuck = 0.1;  //  Load Coral Floor Position
    public static final double kCoralSpit = 0.1;  //  Load Coral Floor Position 
    public final static double kCoralSlowBackward = -0.1; //Move coral slowly in gripper forward
    public final static double kCoralSlowForward = -0.1; //Move coral slowly in gripper backward


    //=======================General======================
    public static final double kDeadband = 0.05;

    //==================Sim Values================
    public static final double kWristRotateGearing = 75.0; //75:1 reduction
    public static final double kWristRotateMomentOfInertia = 0.01; //kg-m^2 - Absolutely no idea
    public static final double kWristLength = 0.1905; //meters - Assume wrist is 15 in. long, pivot is in the center, so actually 7.5 in.
    public static final double kWristMinAngle = 0.0; //radians
    public static final double kWristMaxAngle = 6.28; //2*pi in radians - 360 degrees rotation
    public static final double kWristStartingAngle = 0.0; //radians
    public static final double kWristRollerMomentOfInertia = 0.01; //kg-m^2
    public static final double kWristRollerGearing = 3.0; //3:1 reduction
}
