package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.config.CoralIntakeConfig;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.sim.CoralIntakeSim;

public class CoralIntake extends SubsystemBase{
    //Create Variables
    TalonFX rotateMotor;
    TalonFX spinMotor;
    CANcoder rotateCANcoder;
  
    TalonFXSimState intakePivotSim;
    TalonFXSimState intakeRollerSim;

    VelocityVoltage spinMotorMode;
    PositionVoltage rotateMotorMode;
  
    CoralIntakeSim sim;
  
    public CoralIntake(CoralIntakeConfig config) {
        //Constructor - only runs once

        //Instantiate
        rotateMotor = new TalonFX(config.kRotateCANID, "canivore");
        spinMotor = new TalonFX(config.kSpinCANID, "canivore");
        rotateCANcoder = new CANcoder(config.kRotateCANcoderID);
        spinMotorMode = new VelocityVoltage(CoralIntakeConstants.kSuckSpeed);
        rotateMotorMode = new PositionVoltage(CoralIntakeConstants.kIntakeStowPosition);

        //Fill in the Instantiation
        this.configureMechanism(spinMotor);
        this.configureMechanism(rotateMotor);
        this.configureCancoder(rotateCANcoder);
      
        intakePivotSim = rotateMotor.getSimState();
        intakeRollerSim = spinMotor.getSimState();

        sim = new CoralIntakeSim(config, intakePivotSim, intakeRollerSim);
    }

    public void configureCancoder(CANcoder coralIntakeRotate){       
        //Start Configuring Climber Motor
        CANcoderConfiguration coralIntakeRotateConfig = new CANcoderConfiguration();
        StatusCode coralIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            coralIntakeRotateStatus = coralIntakeRotate.getConfigurator().apply(coralIntakeRotateConfig);
            if (coralIntakeRotateStatus.isOK()) break;
        }
        if (!coralIntakeRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + coralIntakeRotateStatus.toString());
        }
    }

    public void configureMechanism(TalonFX mechanism){     
        //Start Configuring Climber Motor
        TalonFXConfiguration mechanismConfig = new TalonFXConfiguration();
        StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            mechanismStatus = mechanism.getConfigurator().apply(mechanismConfig);
            if (mechanismStatus.isOK()) break;
        }
        if (!mechanismStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
        }

    }

    private void suck() {
        spinMotor.setControl(spinMotorMode.withVelocity(CoralIntakeConstants.kSuckSpeed));
    }

    private void spit() {
        spinMotor.setControl(spinMotorMode.withVelocity(CoralIntakeConstants.kSpitSpeed));
    }

    private void goToPosition(double position) {
        rotateMotor.setControl(rotateMotorMode.withPosition(position));
    }
    
    private double getPivotPosition() {
        return rotateMotor.getPosition().getValueAsDouble();
    }

    private boolean isRotateMotorAtPosition(double position) {
        return ((position - CoralIntakeConstants.kDeadband) <= this.getPivotPosition()) && ((position + CoralIntakeConstants.kDeadband) <= this.getPivotPosition());
    }

    public Command rollerSuck() {
        return run(
            () -> {this.suck();}
        );
    }

    public Command rollerSpit() {
        return run(
            () -> {this.spit();}
        );
    }

    public Command intakeStow() {
        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeStowPosition);}
        );
    }

    public Command intakeRaised() {
        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeRaisedPosition);}
        );
    }

    public Command intakeFloor() {
        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeFloorPosition);}
        );
    }

    public Trigger isIntakeStowed= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeStowPosition);});
    public Trigger isIntakeRaised= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeRaisedPosition);});
    public Trigger isIntakeFloored= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeFloorPosition);});
    
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
        sim.simulationPeriodic();
    }
}
