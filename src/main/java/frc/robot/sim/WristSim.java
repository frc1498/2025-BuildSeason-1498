package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.constants.WristConstants;

public class WristSim implements AutoCloseable{

    TalonFXSimState wristDrive;

    DCMotor elevatorGearbox = DCMotor.getKrakenX60Foc(2);
    ElevatorSim elevate;

    double motRotations;

    Mechanism2d wrist_vis;
    MechanismRoot2d wrist_root;
    MechanismLigament2d wrist_mech;

    public WristSim(ElevatorConfig config, TalonFXSimState wristDrive) {

    }

    public void simulationPeriodic() {
      
    }

    @Override
    public void close() {

    }

}
