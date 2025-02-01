package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    TalonFX intakePivot;
    TalonFX intakeRoller;

    TalonFXSimState intakePivotSim;
    TalonFXSimState intakeRollerSim;

    VelocityVoltage rollerSpit;
    VelocityVoltage rollerSuck;

    PositionVoltage pivot;

    CoralIntakeSim sim;

    public CoralIntake(CoralIntakeConfig config) {
        //Constructor.
        intakePivot = new TalonFX(CoralIntakeConstants.kIntakePivotCANID, "canivore");
        intakeRoller = new TalonFX(CoralIntakeConstants.kIntakeRollerCANID, "canivore");
        rollerSpit = new VelocityVoltage(CoralIntakeConstants.kSpitSpeed);
        rollerSuck = new VelocityVoltage(CoralIntakeConstants.kSuckSpeed);
        pivot = new PositionVoltage(CoralIntakeConstants.kIntakeStowPosition);

        intakePivotSim = intakePivot.getSimState();
        intakeRollerSim = intakeRoller.getSimState();

        sim = new CoralIntakeSim(config, intakePivotSim, intakeRollerSim);
    }

    private void suck() {
        intakeRoller.setControl(rollerSuck);
    }

    private void spit() {
        intakeRoller.setControl(rollerSpit);
    }

    private void goToPosition(double position) {
        intakePivot.setControl(pivot.withPosition(position));
    }
    
    private double getPivotPosition() {
        return intakePivot.getPosition().getValueAsDouble();
    }

    private boolean isIntakeAtPosition(double position) {
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

    public Trigger isIntakeStowed= new Trigger(() -> {return this.isIntakeAtPosition(CoralIntakeConstants.kIntakeStowPosition);});

    public Trigger isIntakeRaised= new Trigger(() -> {return this.isIntakeAtPosition(CoralIntakeConstants.kIntakeRaisedPosition);});

    public Trigger isIntakeFloored= new Trigger(() -> {return this.isIntakeAtPosition(CoralIntakeConstants.kIntakeFloorPosition);});
    
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
