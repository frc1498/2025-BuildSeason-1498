package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class ArmConstants {
public static final int kArmRotateCANID = 13;
public static final int kEncoderCANID = 32;

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

//================Cancoder Settings=============================
public static final int kRotateCancoderRotorToSensorRatio = 0;
public static final SensorDirectionValue kRotateCancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
public static final double AbsoluteSensorDiscontinuityPoint = 1;
public static final double kRotateCancoderOffset = 0;
public static final InvertedValue kRightRotateDirection = InvertedValue.CounterClockwise_Positive;

//===========Coral Positions=============
public static final double kCoralStow = 0.0;
public static final double kCoralLoadFloor = 0.1;
public static final double kCoralLoadHuman = 0.1;
public static final double kCoralL1 = 0.3;
public static final double kCoralL2 = 0.4;
public static final double kCoralL3 = 0.5;
public static final double kCoralL4 = 0.6;

//=============Algae Positions=============
public static final double kAlgaeStow = 0.0;
public static final double kAlgaeLoadFloor = 0.1;  
public static final double kAlgaeLoadL2 = 0.4;
public static final double kAlgaeLoadL3 = 0.5;
public static final double kAlgaeBarge = 0.9;
public static final double kAlgaeProcessor = 0.9;

//==============General===============
public static final double kDeadband = 0.05;

//==================Sim Values================
public static final double kArmRotateGearing = 1.0;
public static final double kArmRotateMomentOfInertia = 0.01; //kg-m^2
public static final double kArmLength = 2.0; //meters
public static final double kArmMinAngle = 0.0; //radians
public static final double kArmMaxAngle = 6.28; //2*pi in radians
public static final double kArmStartingAngle = 0.0; //radians

}