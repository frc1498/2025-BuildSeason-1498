package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ClimberConfig;
import frc.robot.constants.ClimberConstants;
import frc.robot.sim.ClimberSim;

public class Climber extends SubsystemBase{
    //Declare Variables
    TalonFX climberRotate;
    PositionVoltage rotateControl;
    
    TalonFX climberSpin;
    VelocityVoltage spinControl;

    TalonFXSimState climberDriveFrontSim = climberRotate.getSimState();
    ClimberSim sim;
   
    public Climber(ClimberConfig config) {
        //Constructor

        //Instantiate
        climberRotate = new TalonFX(config.kclimberRotateCANID, "canivore");
        rotateControl = new PositionVoltage(ClimberConstants.kClimberStow);

        climberSpin = new TalonFX(config.kclimberRotateCANID, "canivore");
        spinControl = new VelocityVoltage(ClimberConstants.kClimberStop);

        //Fill in Instantiation
        this.configureMechanism(climberRotate, config.climberRotateConfig);
        this.configureMechanism(climberSpin, config.climberSpinConfig);

        SmartDashboard.putData("Climber", this);
    }
   
    public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config){     
        //Start Configuring Climber Motor
        StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            mechanismStatus = mechanism.getConfigurator().apply(config);
            if (mechanismStatus.isOK()) break;
        }
        if (!mechanismStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
        }
    }


    private void climberDriveToPosition(double position) {
            climberRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean isClimberAtPosition(double position) {
            return ((position-ClimberConstants.kDeadband) <= getClimberPosition()) && ((position+ClimberConstants.kDeadband) >= getClimberPosition());
    }

    private double getClimberPosition(){
            return climberRotate.getPosition().getValueAsDouble();       
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
