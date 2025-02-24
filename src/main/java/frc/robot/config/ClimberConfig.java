package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConfig {
    public final int kclimberRotateCANID = 18;

    //Variables
    public TalonFXConfiguration climberRotateConfig = new TalonFXConfiguration();

    //Constructor
    public ClimberConfig(){
        climberRotateConfig = new TalonFXConfiguration();  //Instantiate - make a framework
        this.configureClimberRotate(climberRotateConfig);  //Fill in framework 
    
    }


    public void configureClimberRotate(TalonFXConfiguration rotate) {
        //configure motor
        rotate.CurrentLimits.SupplyCurrentLimit = 40;
        rotate.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
        rotate.MotionMagic.MotionMagicAcceleration = 400;
        rotate.MotionMagic.MotionMagicCruiseVelocity = 90;
        rotate.Voltage.PeakForwardVoltage = 11;
        rotate.Voltage.PeakReverseVoltage = -11;
        rotate.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotate.CurrentLimits.SupplyCurrentLimitEnable = true;
        rotate.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        //Slot 0 Configs
        rotate.Slot0.kP = 5;  // An error of 1 rotation per second results in 2V output
        rotate.Slot0.kI = 0;  // An error of 1 rotation per second increases output by 0.5V every second
        rotate.Slot0.kD = 0;  // A change of 1 rotation per second squared results in 0.01 volts output
        rotate.Slot0.kG = 0;
        rotate.Slot0.kV = .125;  // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        rotate.Slot0.kA = .008;
        rotate.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    }

}
