package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class WristConfig {
    public final int kRotateCANID = 14;  //Rotate Motor
    public final int kEncoderCANID = 33;  //Throughbore encoder
    public final int kSpinCANID = 15;  //Rotate Motor
    public final int kRangeCANID = 36; //Reef Distance CANrange

    public final int kBeamBreakGripperFront = 2; //Gripper Front DIO
    public final int kBeamBreakGripperRear = 1; //Gripper Rear DIO

    public CANcoderConfiguration wristRotateCANcoderConfig;  //Create variable of type CANcoderConfiguration
    public TalonFXConfiguration wristRotateConfig;  //Create variable of type CANcoderConfiguration
    public TalonFXConfiguration wristSpinConfig;  //Create variable of type CANcoderConfiguration
    public CANrangeConfiguration canRangeConfig;  //Create variable of type CanRangeConfig


     public class WristRotateConfig {

    }


    //Constructor
    public WristConfig() {
        wristRotateCANcoderConfig = new CANcoderConfiguration();  //Instantiate - make a framework
        this.configureWristRotateCANcoder(wristRotateCANcoderConfig);  //Fill in framework
  
        wristRotateConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureWristRotate(wristRotateConfig);  //Fill in framework

        wristSpinConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureWristSpin(wristSpinConfig);  //Fill in framework

        canRangeConfig = new CANrangeConfiguration(); //Instantiate - make a framework
        this.configureCANRange(canRangeConfig); //Fill in framework
    }

    public void configureCANRange(CANrangeConfiguration canconfig){
        //configure the canRange
        canconfig.FovParams.FOVRangeX = 6.75;
        canconfig.FovParams.FOVRangeY = 6.75;
        canconfig.ProximityParams.MinSignalStrengthForValidMeasurement=2500;
    }


    public void configureWristRotate(TalonFXConfiguration rotate){

        //configure motor
        rotate.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotate.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotate.MotorOutput.PeakForwardDutyCycle = 1;
        rotate.MotorOutput.PeakReverseDutyCycle = -1;

        rotate.CurrentLimits.StatorCurrentLimit = 120.0;
        rotate.CurrentLimits.StatorCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLimit = 20; //was 20
        rotate.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLowerLimit = 5;
        rotate.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        rotate.Slot0.kP = 60;  // An error of 1 rotation per second results in 2V output
        rotate.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotate.Slot0.kD = 2;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotate.Slot0.kS = 0;
        rotate.Slot0.kV = 2.5;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rotate.Slot0.kA = 0;
        rotate.Slot0.kG = 0;
        rotate.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        rotate.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;




        rotate.Voltage.PeakForwardVoltage = 11;
        rotate.Voltage.PeakReverseVoltage = -11;

        //Fuse the Cancoder here
        rotate.Feedback.FeedbackRemoteSensorID = 33;
        rotate.Feedback.FeedbackRotorOffset = 0;
        rotate.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        rotate.Feedback.RotorToSensorRatio = 25;
        rotate.Feedback.SensorToMechanismRatio = 1;

        rotate.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        rotate.MotionMagic.MotionMagicCruiseVelocity = 2;
        rotate.MotionMagic.MotionMagicAcceleration = 6.67;
        rotate.MotionMagic.MotionMagicJerk = 66.7;

        rotate.Audio.AllowMusicDurDisable = true;

    }

    public void configureWristSpin(TalonFXConfiguration spin){
        
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
        spin.Slot0.kP = 0.45; // An error of 1 rotation per second results in 2V output
        spin.Slot0.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
        spin.Slot0.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
        spin.Slot0.kV = 0.132; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
      
        //Slot 1 Configs
        spin.Slot1.kP = 10; // An error of 1 rotation per second results in 2V output
        spin.Slot1.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
        spin.Slot1.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
        spin.Slot1.kV = 0; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second

        //Slot 2 Configs
        spin.Slot2.kP = 0; // An error of 1 rotation per second results in 2V output
        spin.Slot2.kI = 0; // An error of 1 rotation per second increases output by 0.5V every second
        spin.Slot2.kD = 0; // A change of 1 rotation per second squared results in 0.01 volts output
        spin.Slot2.kV = 0.132; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second


    }

    public void configureWristRotateCANcoder(CANcoderConfiguration CANcoderConfig){
        CANcoderConfig.MagnetSensor.MagnetOffset = -0.234130859375;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
  



}
