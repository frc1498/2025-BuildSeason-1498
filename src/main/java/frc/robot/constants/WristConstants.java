package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class WristConstants {

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
    public static final double kAlgaeL2= 0.1; // Load Algae L2 Position
    public static final double kAlgaeL3 = 0.1; // Load Algae L3 Position 
    public static final double kAlgaeBarge = 0.9; //Score position for Algae in the Barge
    public static final double kAlgaeProcessor = 0.1; //Score position for Algae in the processor

    //===============Wrist Intake Spin Velocities======================
    public static final double kCoralStop = 0.0;  //Stow Position 
    public static final double kCoralSuck = 0.1;  //  Load Coral Floor Position
    public static final double kCoralSpit = 0.1;  //  Load Coral Floor Position 

    //=======================General======================
    public static final double kDeadband = 0.05;

}
