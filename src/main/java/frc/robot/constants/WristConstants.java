package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class WristConstants {
    public static final int kWristRotateCANID = 13;  //Rotate Motor
    public static final int kWristEncoderCANID = 32;  //Throughbore encoder

    //===============Motor Constants for a Motion Magic Position Controlled Motor======================
    public static final InvertedValue kRotateDirection = InvertedValue.Clockwise_Positive;
    public static final double kRotateProportional = 0; 
    public static final double kRotateIntegral = 0;
    public static final double kRotateDerivative = 0;
    public static final double kRotateGravity = 0; 
    public static final double kRotateVelocityFeedFoward = 0;
    public static final int kRotateMaxForwardVoltage = 11;
    public static final int kRotateMaxReverseVoltage = -11;
    public static final double kRotateMotionMagicCruiseVelocity = 0;
    public static final double kRotateMotionMagicAcceleration = 0;
    public static final int kRotateMotionMagicJerk = 0;
    public static final int kRotateSupplyCurrentLimit = 0;
    public static final int kRotateVoltageClosedLoopRampPeriod = 0;
    public static final boolean kRotateBrake = true;
    public static final boolean kRotateSupplyCurrentLimitEnable = true;

    //================WristRotate Cancoder Settings=============================
    public static final int kRotateCancoderRotorToSensorRatio = 0;
    public static final SensorDirectionValue kRotateCancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double AbsoluteSensorDiscontinuityPoint = 1;
    public static final double kRotateCancoderOffset = 0;
    public static final InvertedValue kRightRotateDirection = InvertedValue.CounterClockwise_Positive;


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

    //=======================General======================
    public static final double kDeadband = 0.05;

}
