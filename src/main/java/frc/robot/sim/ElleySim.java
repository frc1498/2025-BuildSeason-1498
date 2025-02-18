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

    DCMotor elevatorGearbox = DCMotor.getKrakenX60Foc(2);
    ElevatorSim elevate;

    double motRotations;
    private double outputRotations;
    private double outputRotationsPerSec;
    private double inputRotations;
    private double inputRotationsPerSec;

    Mechanism2d elevator_vis;
    MechanismRoot2d elevator_root;
    MechanismLigament2d elevator_mech;

    public ElleySim(ElevatorConfig config, TalonFXSimState leftDrive) {
        this.leftDrive = leftDrive;

        this.elevate = new ElevatorSim(
            elevatorGearbox, 
            ElevatorConstants.kElevatorGearing, 
            ElevatorConstants.kElevatorCarriageMass, 
            ElevatorConstants.kElevatorDrumRadius, 
            ElevatorConstants.kElevatorMinHeight, 
            ElevatorConstants.kElevatorMaxHeight, 
            false, 
            ElevatorConstants.kElevatorStartingHeight);

            elevator_vis = new Mechanism2d(20, 20);
            elevator_root = elevator_vis.getRoot("Elevator Root", 10, 0);
            //Multiply elevator height by 5 to make it more visible.
            elevator_mech = elevator_root.append(new MechanismLigament2d("Elevator", elevate.getPositionMeters() * 5, 90));
        
        SmartDashboard.putData("ELERVATER", elevator_vis);
    }


    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        leftDrive.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        elevate.setInput(leftDrive.getMotorVoltage());
        elevate.update(0.02);

        //Update sensor positions.

        //Convert elevator position to output rotations.
        outputRotations = elevate.getPositionMeters() / (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
        outputRotationsPerSec = elevate.getVelocityMetersPerSecond() / (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);

        //Convert output rotations to input rotations
        inputRotations = this.outputRotToInputRot(outputRotations, ElevatorConstants.kElevatorGearing);
        inputRotationsPerSec = this.outputRotToInputRot(outputRotationsPerSec, ElevatorConstants.kElevatorGearing);

        leftDrive.setRawRotorPosition(inputRotations);
        leftDrive.setRotorVelocity(inputRotationsPerSec);
        elevator_mech.setLength(elevate.getPositionMeters());
    }

    private double radToDeg(double radians) {
        return radians / (2 * Math.PI) * 360.0;
    }

    private double degToRot(double degrees) {
        return degrees / 360.0;
    }

    private double inputRotToOutputRot(double inputRotations, double gearing) {
        return inputRotations / gearing;
    }

    private double outputRotToInputRot(double outputRotations, double gearing) {
        return outputRotations * gearing;
    }

    @Override
    public void close() {
        elevator_vis.close();
    }
}
