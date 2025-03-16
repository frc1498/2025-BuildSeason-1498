package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class CoralIntakeConfig {
    public final int kRotateCANID = 9;
    public final int kSpinCANID = 10;

    public final int kRotateCANcoderID = 31;

    public final int kBeamBreakIntake = 0;  //Intake Beambreak DIO


    public CANcoderConfiguration coralIntakeCANcoderConfig;  //Create variable of type CANcoderConfiguration
    public TalonFXConfiguration coralIntakeRotateConfig;  //Create variable of type CANcoderConfiguration
    public TalonFXConfiguration coralIntakeSpinConfig;  //Create variable of type CANcoderConfiguration



    //Constructor
    public CoralIntakeConfig() {
        coralIntakeCANcoderConfig = new CANcoderConfiguration();  //Instantiate - make a framework
        this.configureCoralIntakeCANcoder(coralIntakeCANcoderConfig);  //Fill in framework
  
        coralIntakeRotateConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureCoralIntakeRotate(coralIntakeRotateConfig);  //Fill in framework

        coralIntakeSpinConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureCoralIntakeSpin(coralIntakeSpinConfig);  //Fill in framework
    }

    public void configureCoralIntakeRotate(TalonFXConfiguration rotate){

        //configure motor
        rotate.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotate.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotate.MotorOutput.PeakForwardDutyCycle = 1;
        rotate.MotorOutput.PeakReverseDutyCycle = -1;

        rotate.CurrentLimits.StatorCurrentLimit = 120.0;
        rotate.CurrentLimits.StatorCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLimit = 20;  //was 40
        rotate.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        rotate.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        rotate.Slot0.kP = 200;  // An error of 1 rotation per second results in 2V output
        rotate.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotate.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotate.Slot0.kS = 0;
        rotate.Slot0.kV = 15;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rotate.Slot0.kA = 0.888;
        rotate.Slot0.kG = 0.2;
        rotate.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        rotate.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        rotate.Voltage.PeakForwardVoltage = 11;
        rotate.Voltage.PeakReverseVoltage = -11;

        //Fuse the Cancoder here
        rotate.Feedback.FeedbackRemoteSensorID = 31;
        rotate.Feedback.FeedbackRotorOffset = 0;
        rotate.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;  //Changed to Fused CANcoder, because the mechanism has tensioner.
        rotate.Feedback.SensorToMechanismRatio = 1;
        rotate.Feedback.RotorToSensorRatio = 150;

        rotate.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        rotate.MotionMagic.MotionMagicCruiseVelocity = 0.8;

        
        rotate.MotionMagic.MotionMagicAcceleration = 4;
        rotate.MotionMagic.MotionMagicJerk = 22.22;

        rotate.Audio.AllowMusicDurDisable = true;
    }

    public void configureCoralIntakeSpin(TalonFXConfiguration spin){
        
        //Configure Motor
        spin.CurrentLimits.SupplyCurrentLimit = 40;
        spin.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        spin.Voltage.PeakForwardVoltage = -11;
        spin.Voltage.PeakReverseVoltage = 11;
        spin.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        spin.CurrentLimits.SupplyCurrentLimitEnable = true;
        spin.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spin.Audio.AllowMusicDurDisable = true;

        //Slot 0 Configs
        spin.Slot0.kP = 0; // An error of 1 rotation per second results in 2V output
        spin.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
        spin.Slot0.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
        spin.Slot0.kV = 0.14; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
      
    }

    public void configureCoralIntakeCANcoder(CANcoderConfiguration CANcoderConfig){
        CANcoderConfig.MagnetSensor.MagnetOffset = -0.23876953125;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
  

}
