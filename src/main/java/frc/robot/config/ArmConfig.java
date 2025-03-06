package frc.robot.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class ArmConfig {
    //Contants
    public static final int kArmRotateCANID = 13;
    public static final int kEncoderCANID = 32;

    //Variables
    public CANcoderConfiguration armRotateCANcoderConfig;  //Create variable of type CANcoderConfiguration
    public TalonFXConfiguration armRotateConfig;  //Create variable of type TalonFXConfiguration

    //Constructor    
    public ArmConfig(){   
        armRotateConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureArmRotate(armRotateConfig);  //Fill in framework 

        armRotateCANcoderConfig = new CANcoderConfiguration();  //Instantiate - make a framework
        this.configureArmRotateCANcoder(armRotateCANcoderConfig);  //Fill in framework
    }


    public void configureArmRotate(TalonFXConfiguration rotate){

        //configure motor
        rotate.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rotate.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotate.MotorOutput.PeakForwardDutyCycle = 1;
        rotate.MotorOutput.PeakReverseDutyCycle = -1;

        rotate.CurrentLimits.StatorCurrentLimit = 120.0;
        rotate.CurrentLimits.StatorCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLimit = 20;
        rotate.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotate.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
        rotate.CurrentLimits.SupplyCurrentLowerTime = 1;

        //Slot 0 Configs
        rotate.Slot0.kP = 60;  // An error of 1 rotation per second results in 2V output
        rotate.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotate.Slot0.kD = 5;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotate.Slot0.kS = 0;
        rotate.Slot0.kV = 21;  // KV for a Kraken X60 is 490 rpm/V. 490/60 is 8.1667 rps/V.  The inverse is 0.122449 V/rps.
        rotate.Slot0.kA = 1;
        rotate.Slot0.kG = 0.223;
        rotate.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        rotate.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        rotate.Voltage.PeakForwardVoltage = 11;
        rotate.Voltage.PeakReverseVoltage = -11;

        //Fuse the Cancoder here
        rotate.Feedback.FeedbackRemoteSensorID = 32;
        rotate.Feedback.FeedbackRotorOffset = 0;
        rotate.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        rotate.Feedback.RotorToSensorRatio = 105.883;
        rotate.Feedback.SensorToMechanismRatio = 1;

        rotate.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        
        rotate.MotionMagic.MotionMagicCruiseVelocity = 0.85;

        rotate.MotionMagic.MotionMagicAcceleration = 3.5;
        rotate.MotionMagic.MotionMagicJerk = 47.6;

        rotate.Audio.AllowMusicDurDisable = true;

    }

    public void configureArmRotateCANcoder(CANcoderConfiguration CANcoderConfig){
        CANcoderConfig.MagnetSensor.MagnetOffset = 0.11865234375;
        CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }

}
