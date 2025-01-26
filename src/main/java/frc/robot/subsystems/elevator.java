package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ElevatorConfig;
import frc.robot.constants.ElevatorConstants;
import frc.robot.sim.ElleySim;

public class Elevator extends SubsystemBase {
    TalonFX elevatorDriveFront = new TalonFX(ElevatorConstants.kElevatorDriveFrontCANID, "canivore");
    TalonFX elevatorDriveBack = new TalonFX(ElevatorConstants.kElevatorDriveBackCANID, "canivore");

    TalonFXSimState elevatorDriveFrontSim = elevatorDriveFront.getSimState();
    TalonFXSimState elevatorDriveBackSim = elevatorDriveBack.getSimState();

    PositionVoltage posControl;
    
    ElevatorConfig config;
    ElleySim sim;

    public Elevator(ElevatorConfig config) {
        //Constructor.
        elevatorDriveFront.getConfigurator().apply(config.frontConfig);
        elevatorDriveBack.getConfigurator().apply(config.backConfig);
        this.config = config;
        sim = new ElleySim(config, elevatorDriveFrontSim, elevatorDriveBackSim);
        posControl = new PositionVoltage(0);
    }

    private void elevatorDriveToPosition(double position) {
        elevatorDriveFront.setControl(posControl.withPosition(position));
    }

    private double getCurrentPosition() {
        return elevatorDriveFront.getPosition().getValueAsDouble();
    }

    private boolean isElevatorAtPosition(double position) {
        return (position - ElevatorConstants.kDeadband) <= this.getCurrentPosition() && (position + ElevatorConstants.kDeadband) >= this.getCurrentPosition();
    }
    
    public Command elevatorHome() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kHome);}
            ).until(this.isElevatorHome);
    }

    public Command elevatorL1() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kL1Position);}
        ).until(this.isElevatorL1);
    }

    public Command elevatorL2() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kL2Position);}
        ).until(this.isElevatorL2);
    }
    
    public Command elevatorL3() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kL3Position);}
        ).until(this.isElevatorL3);
    }

    public Command elevatorL4() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kL4Position);}
        ).until(this.isElevatorL4);
    }

    //This *feels* too complicated, but it works.
    public Command elevatorPosition(Supplier<Double> position) {
        return run(
            () -> {this.elevatorDriveToPosition(position.get());}
        );
    }

    public final Trigger isElevatorHome = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kHome);});

    public final Trigger isElevatorL1 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kL1Position);});

    public final Trigger isElevatorL2 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kL2Position);});

    public final Trigger isElevatorL3 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kL3Position);});

    public final Trigger isElevatorL4 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kL4Position);});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.      
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void simulationInit() {
        sim = new ElleySim(config, elevatorDriveFrontSim, elevatorDriveBackSim);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}