package frc.robot.config;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConfig {

    public final int kElevatorDriveFrontCANID = 11;
    public final int kElevatorDriveBackCANID = 12;

    public TalonFXConfiguration frontConfig = new TalonFXConfiguration();
    public TalonFXConfiguration backConfig = new TalonFXConfiguration();

    public ClosedLoopGeneralConfigs frontGains;
    
    public void configureElevatorRotate(TalonFXConfiguration rotate){

        //configure motor
        rotate.CurrentLimits.SupplyCurrentLimit = 0;
        rotate.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        rotate.MotionMagic.MotionMagicAcceleration = 0;
        rotate.MotionMagic.MotionMagicCruiseVelocity = 0;
        rotate.Voltage.PeakForwardVoltage = 11;
        rotate.Voltage.PeakReverseVoltage = -11;
        rotate.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotate.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotate.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Slot 0 Configs
        rotate.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        rotate.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotate.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotate.Slot0.kG = 0;
        rotate.Slot0.kV = 0;  // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        rotate.Slot0.GravityType = GravityTypeValue.Elevator_Static;


        //Fuse the Cancoder here
        rotate.Feedback.FeedbackRemoteSensorID = 0;
        rotate.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        rotate.Feedback.SensorToMechanismRatio = 0;
        rotate.Feedback.RotorToSensorRatio = 0;

    }
}
