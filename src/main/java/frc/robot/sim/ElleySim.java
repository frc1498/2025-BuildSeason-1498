package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ElevatorConfig;
import frc.robot.constants.ElevatorConstants;

/**
 * I have to name this 'ElleySim' for now because 'ElevatorSim' is a class from WPI that I'm actually trying to use.
 */
public class ElleySim implements AutoCloseable{

    TalonFXSimState leftDrive;
    TalonFXSimState rightDrive;

    DCMotor elevatorGearbox = DCMotor.getKrakenX60Foc(2);
    ElevatorSim elevate;

    double motRotations;

    Mechanism2d elevator_vis;
    MechanismRoot2d elevator_root;
    MechanismLigament2d elevator_mech;

    public ElleySim(ElevatorConfig config, TalonFXSimState leftDrive, TalonFXSimState rightDrive) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;

        this.elevate = new ElevatorSim(
            elevatorGearbox, 
            ElevatorConstants.kElevatorGearing, 
            ElevatorConstants.kElevatorCarriageMass, 
            ElevatorConstants.kElevatorDrumRadius, 
            ElevatorConstants.kElevatorMinHeight, 
            ElevatorConstants.kElevatorMaxHeight, 
            false, 
            ElevatorConstants.kElevatorStartingHeight);

            elevator_vis = new Mechanism2d(20, 7);
            elevator_root = elevator_vis.getRoot("Elevator Root", 10, 0);
            elevator_mech = elevator_root.append(new MechanismLigament2d("Elevator", elevate.getPositionMeters(), 90));
        
        SmartDashboard.putData("ELERVATER", elevator_vis);
    }


    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        leftDrive.setSupplyVoltage(12);
        rightDrive.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        elevate.setInput(leftDrive.getMotorVoltage());
        elevate.update(0.02);

        //Update sensor positions.

        //rotations = meters * gear ratio / drum radius
        motRotations = (elevate.getPositionMeters() * ElevatorConstants.kElevatorGearing) / ElevatorConstants.kElevatorDrumRadius;

        leftDrive.setRawRotorPosition(motRotations * ElevatorConstants.kElevatorGearing);
        rightDrive.setRawRotorPosition(motRotations * ElevatorConstants.kElevatorGearing);
        leftDrive.setRotorVelocity(motRotations * ElevatorConstants.kElevatorGearing);
        rightDrive.setRotorVelocity(motRotations * ElevatorConstants.kElevatorGearing);

        elevator_mech.setLength(elevate.getPositionMeters());
    }

    @Override
    public void close() {
        elevator_vis.close();
    }
}
