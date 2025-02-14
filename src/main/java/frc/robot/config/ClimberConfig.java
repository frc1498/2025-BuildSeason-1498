package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConfig {
    public final int kclimberRotateCANID = 18;
    public final int kclimberSpinCANID = 19;

    //Variables
    public TalonFXConfiguration climberRotateConfig = new TalonFXConfiguration();
    public TalonFXConfiguration climberSpinConfig = new TalonFXConfiguration();

    //Constructor
    public ClimberConfig(){
        climberRotateConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureClimberRotate(climberRotateConfig);  //Fill in framework 

        climberSpinConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureClimberSpin(climberSpinConfig);  //Fill in framework 
    
    }


    public void configureClimberRotate(TalonFXConfiguration rotate) {
        //configure motor
        rotate.CurrentLimits.SupplyCurrentLimit = 40;
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
        rotate.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    }

    public void configureClimberSpin(TalonFXConfiguration spin) {
        //configure motor
        spin.CurrentLimits.SupplyCurrentLimit = 40;
        spin.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        spin.MotionMagic.MotionMagicAcceleration = 0;
        spin.MotionMagic.MotionMagicCruiseVelocity = 0;
        spin.Voltage.PeakForwardVoltage = 11;
        spin.Voltage.PeakReverseVoltage = -11;
        spin.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        spin.CurrentLimits.SupplyCurrentLimitEnable = true;
        spin.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Slot 0 Configs
        spin.Slot0.kP = 0;  // An error of 1 rotation per second results in 2V output
        spin.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        spin.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        spin.Slot0.kG = 0;
        spin.Slot0.kV = 0;  // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        spin.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    }

}
