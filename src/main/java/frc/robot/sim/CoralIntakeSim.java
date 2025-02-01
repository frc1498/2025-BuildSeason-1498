package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.CoralIntakeConfig;
import frc.robot.constants.CoralIntakeConstants;

public class CoralIntakeSim implements AutoCloseable{

    TalonFXSimState intakePivot;
    TalonFXSimState intakeSpin;

    DCMotor intakeGearbox = DCMotor.getKrakenX60Foc(1);
    DCMotor intakeRoller = DCMotor.getKrakenX60Foc(1);
    
    SingleJointedArmSim intake;
    FlywheelSim intakeRolly;

    double motRotations;

    Mechanism2d intake_vis;
    MechanismRoot2d intake_root;
    MechanismLigament2d intake_rotate_mech;
    MechanismLigament2d intake_roller_mech;

    public CoralIntakeSim(CoralIntakeConfig config, TalonFXSimState intakePivot, TalonFXSimState intakeSpin) {
        this.intakePivot = intakePivot;
        this.intakeSpin = intakeSpin;

        this.intake = new SingleJointedArmSim(
            intakeGearbox, 
            CoralIntakeConstants.kIntakeRotateGearing, 
            CoralIntakeConstants.kIntakeRotateMomentOfInertia, 
            CoralIntakeConstants.kIntakeLength, 
            CoralIntakeConstants.kIntakeMinAngle, 
            CoralIntakeConstants.kIntakeMaxAngle, 
            false, 
            CoralIntakeConstants.kIntakeStartingAngle);

        this.intakeRolly = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                intakeRoller, 
                CoralIntakeConstants.kIntakeRollerMomentOfInertia, 
                CoralIntakeConstants.kIntakeRollerGearing), 
            intakeRoller);

        intake_vis = new Mechanism2d(20, 20);
        intake_root = intake_vis.getRoot("Intake Root", 10, 10);
        intake_rotate_mech = intake_root.append(new MechanismLigament2d("Intake", CoralIntakeConstants.kIntakeLength, intake.getAngleRads()));
        intake_roller_mech = intake_root.append(new MechanismLigament2d("Intake Roller", 0.5, 90));

        SmartDashboard.putData("INTAKE", intake_vis);
    }

    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        intakePivot.setSupplyVoltage(12);
        intakeSpin.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        intake.setInput(intakePivot.getMotorVoltage());
        intakeRolly.setInput(intakeSpin.getMotorVoltage());
        intake.update(0.02);
        intakeRolly.update(0.02);

        //Update sensor positions.

        //rotations = (radians / 2*pi) * gear ratio
        motRotations = (intake.getAngleRads() * CoralIntakeConstants.kIntakeRotateGearing);

        intakePivot.setRawRotorPosition(motRotations * CoralIntakeConstants.kIntakeRotateGearing);
        intakeSpin.setRawRotorPosition(motRotations * CoralIntakeConstants.kIntakeRotateGearing);
        intakePivot.setRotorVelocity(motRotations * CoralIntakeConstants.kIntakeRotateGearing);
        intakeSpin.setRotorVelocity(motRotations * CoralIntakeConstants.kIntakeRotateGearing);

        intake_rotate_mech.setAngle(intake.getAngleRads());
        intake_roller_mech.setAngle(90);      
    }

    @Override
    public void close() {
        intake_vis.close();
    }

}
