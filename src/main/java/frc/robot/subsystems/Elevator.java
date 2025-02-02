package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    //Declare Variables
    TalonFX elevatorDriveFront;
    TalonFX elevatorDriveRear;
  
    TalonFXSimState elevatorDriveFrontSim;
    TalonFXSimState elevatorDriveRearSim;
  
    PositionVoltage posControl;

    ElevatorConfig config;
    ElleySim sim;

    public Elevator(ElevatorConfig config) {
        //Constructor.

        //Instantiate
        elevatorDriveFront = new TalonFX(config.kElevatorDriveFrontCANID, "canivore");
        elevatorDriveRear = new TalonFX(config.kElevatorDriveBackCANID, "canivore");

        //Fill In the Instantiation
        this.configureMechanism(elevatorDriveFront);
        this.configureMechanism(elevatorDriveRear);

        this.config = config;
      
        elevatorDriveFrontSim = elevatorDriveFront.getSimState();
        elevatorDriveRearSim = elevatorDriveRear.getSimState();
      
        sim = new ElleySim(config, elevatorDriveFrontSim, elevatorDriveRearSim);
      
        posControl = new PositionVoltage(0);
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

    //=================Private Commands======================
    private void elevatorDriveToPosition(double position) {
        elevatorDriveFront.setControl(posControl.withPosition(position));
    }

    private double getCurrentPosition() {
        return elevatorDriveFront.getPosition().getValueAsDouble();
    }

    private boolean isElevatorAtPosition(double position) {
        return (position - ElevatorConstants.kDeadband) <= this.getCurrentPosition() && (position + ElevatorConstants.kDeadband) >= this.getCurrentPosition();
    }

    //=====================Public Commands==================================
    public Command elevatorCoralStow() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralStow);}
        ).until(this.isElevatorCoralStow);
    }

    public Command elevatorCoralLoadFloor() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralLoadFloor);}
        ).until(this.isElevatorCoralLoadFloor);
    }

    public Command elevatorCoralLoadHuman() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralLoadHuman);}
        ).until(this.isElevatorCoralLoadHuman);
    }

    public Command elevatorCoralL1() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL1);}
        ).until(this.isElevatorCoralL1);
    }

    public Command elevatorCoralL2() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL2);}
        ).until(this.isElevatorCoralL2);
    }
    
    public Command elevatorCoralL3() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL3);}
        ).until(this.isElevatorCoralL3);
    }

    public Command elevatorCoralL4() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL4);}
        ).until(this.isElevatorCoralL4);
    }

    public Command elevatorAlgaeStow() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeStow);}
        ).until(this.isElevatorAlgaeStow);
    }

    public Command elevatorAlgaeLoadFloor() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeLoadFloor);}
        ).until(this.isElevatorAlgaeLoadFloor);
    }

    public Command elevatorAlgaeL2() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeL2);}
        ).until(this.isElevatorAlgaeL2);
    }
    
    public Command elevatorAlgaeL3() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeL3);}
        ).until(this.isElevatorAlgaeL3);
    }
    
    public Command elevatorAlgaeBarge() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeBarge);}
        ).until(this.isElevatorAlgaeBarge);
    }

    public Command elevatorAlgaeProcessor() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeProcessor);}
        ).until(this.isElevatorAlgaeProcessor);
    }

    //This *feels* too complicated, but it works.
    public Command elevatorPosition(Supplier<Double> position) {
        return run(
            () -> {this.elevatorDriveToPosition(position.get());}
        );
    }

    //========Triggers for Elevator Coral=========
    public final Trigger isElevatorCoralStow = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralStow);});
    public final Trigger isElevatorCoralLoadFloor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralLoadFloor);});
    public final Trigger isElevatorCoralLoadHuman = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralLoadHuman);});
    public final Trigger isElevatorCoralL1 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL1);});
    public final Trigger isElevatorCoralL2 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL2);});
    public final Trigger isElevatorCoralL3 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL3);});
    public final Trigger isElevatorCoralL4 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL4);});

    //=========Triggers for Elevator Algae========
    public final Trigger isElevatorAlgaeStow = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeStow);});
    public final Trigger isElevatorAlgaeLoadFloor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeLoadFloor);});
    public final Trigger isElevatorAlgaeL2 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeL2);});
    public final Trigger isElevatorAlgaeL3 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeL3);});
    public final Trigger isElevatorAlgaeBarge = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeBarge);});
    public final Trigger isElevatorAlgaeProcessor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeProcessor);});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.      
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void simulationInit() {
        sim = new ElleySim(config, elevatorDriveFrontSim, elevatorDriveRearSim);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
