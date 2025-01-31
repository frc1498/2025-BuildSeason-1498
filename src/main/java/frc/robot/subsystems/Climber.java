package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ClimberConfig;
import frc.robot.constants.ClimberConstants;
import frc.robot.sim.ClimberSim;

public class Climber extends SubsystemBase{
    TalonFX climberRotate;
    CANcoder climberRotateEncoder;
    PositionVoltage rotateControl;

    PositionVoltage posControl;
    
    TalonFXSimState climberDriveFrontSim = climberRotate.getSimState();

    ClimberConfig config;
    ClimberSim sim;

   
    public Climber(ClimberConfig config) {
        //Constructor
        //rotateControl = new PositionVoltage(ClimberConstants.kCoralStow);

        climberRotate.getConfigurator().apply(config.ClimberConfig);
        this.config = config;
        //sim = new ClimberSim(config, climberDriveFrontSim);
        posControl = new PositionVoltage(0);
    }
    
    //=============================================================
    //====================Private Methods==========================
    //=============================================================
    private void climberDriveToPosition(double position) {
            climberRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean isClimberAtPosition(double position) {
            return ((position-ClimberConstants.kDeadband) <= getClimberPosition()) && ((position+ClimberConstants.kDeadband) >= getClimberPosition());
    }

    private double getClimberPosition(){
            return climberRotate.getPosition().getValueAsDouble();       
    }

    //===================================================
    //=====================Public Methods===============
    //===================================================    
    public void configureClimberRotate(TalonFX climberRotate){
        //Start Configuring Climber Motor
        TalonFXConfiguration climberRotateConfig = new TalonFXConfiguration();

        climberRotateConfig.MotorOutput.Inverted = ClimberConstants.kRotateDirection;
        climberRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberRotateConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.kRotateSupplyCurrentLimit;
        climberRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ClimberConstants.kRotateVoltageClosedLoopRampPeriod;
        climberRotateConfig.Voltage.PeakForwardVoltage = ClimberConstants.kRotateMaxForwardVoltage;
        climberRotateConfig.Voltage.PeakReverseVoltage = ClimberConstants.kRotateMaxReverseVoltage;
        climberRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //Configure Gains and Motion Magic Items
        Slot0Configs slot0 = climberRotateConfig.Slot0;
        slot0.kP = ClimberConstants.kRotateProportional;
        slot0.kI = ClimberConstants.kRotateIntegral;
        slot0.kD = ClimberConstants.kRotateDerivative;
        //slot0.GravityType = ;  //We probably don't need this for the climber
        slot0.kV = ClimberConstants.kRotateVelocityFeedFoward;
        //slot0.kS = ClimberConstants.kClimberStaticFeedFoward;  //Probably don't need this for the climber?

        //Setting the config option that allows playing music on the motor during disabled.
        climberRotateConfig.Audio.AllowMusicDurDisable = true;
 
        StatusCode climberRotateStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            climberRotateStatus = climberRotate.getConfigurator().apply(climberRotateConfig);
            if (climberRotateStatus.isOK()) break;
        }
        if (!climberRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + climberRotateStatus.toString());
        }
        //m_LeftClimb.setPosition(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
