package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class ClimberConstants {
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
  
    //=======================General======================
    public static final double kDeadband = 0.05;
}
