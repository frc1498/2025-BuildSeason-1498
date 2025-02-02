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
import frc.robot.config.WristConfig;
import frc.robot.constants.WristConstants;

public class WristSim implements AutoCloseable{

    TalonFXSimState wristDrive;
    TalonFXSimState wristSpinny;

    DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);
    DCMotor wristRoll = DCMotor.getKrakenX60Foc(1);
    
    SingleJointedArmSim wrist;
    FlywheelSim wristRoller;

    double motRotations;
    double motSpeed;
    double rollRotations;

    Mechanism2d wrist_vis;
    MechanismRoot2d wrist_root;
    MechanismLigament2d wrist_rotate_mech;
    MechanismLigament2d wrist_roller_mech;

    public WristSim(WristConfig config, TalonFXSimState wristDrive, TalonFXSimState wristRolly) {
        this.wristDrive = wristDrive;
        this.wristSpinny = wristRolly;

        this.wrist = new SingleJointedArmSim(
            wristGearbox, 
            WristConstants.kWristRotateGearing, 
            WristConstants.kWristRotateMomentOfInertia, 
            WristConstants.kWristLength, 
            WristConstants.kWristMinAngle, 
            WristConstants.kWristMaxAngle, 
            true, 
            WristConstants.kWristStartingAngle);

        this.wristRoller = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                wristRoll, 
                WristConstants.kWristRollerMomentOfInertia, 
                WristConstants.kWristRollerGearing), 
            wristRoll);

        wrist_vis = new Mechanism2d(20, 20);
        wrist_root = wrist_vis.getRoot("Wrist Root", 10, 10);
        wrist_rotate_mech = wrist_root.append(new MechanismLigament2d("Wrist", WristConstants.kWristLength, this.radToAngle(wrist.getAngleRads())));
        wrist_roller_mech = wrist_root.append(new MechanismLigament2d("Wrist Roller", 0.5, 0));

        SmartDashboard.putData("WREEST", wrist_vis);
    }

    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        wristDrive.setSupplyVoltage(12);
        wristSpinny.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        wrist.setInput(wristDrive.getMotorVoltage());
        wristRoller.setInput(wristSpinny.getMotorVoltage());
        wrist.update(0.02);
        wristRoller.update(0.02);

        //Update sensor positions.

        //rotations = (radians / 2*pi) * gear ratio
        motRotations = this.radToAngle(wrist.getAngleRads()) / WristConstants.kWristRotateGearing;
        motSpeed = this.radToAngle(wrist.getVelocityRadPerSec()) / WristConstants.kWristRotateGearing;
        rollRotations = wristRoller.getAngularVelocityRPM() / 60;

        wristDrive.setRawRotorPosition(motRotations);
        wristDrive.setRotorVelocity(motSpeed);
        wristSpinny.setRotorVelocity(rollRotations);

        wrist_rotate_mech.setAngle(this.radToAngle(wrist.getAngleRads()));
        wrist_roller_mech.setAngle(0);      
    }

    @Override
    public void close() {
        wrist_vis.close();
    }
    
    private double radToAngle(double radians) {
        return (radians / (2 * Math.PI)) * 360;
    }
}
