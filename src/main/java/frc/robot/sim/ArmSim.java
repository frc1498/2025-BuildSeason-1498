package frc.robot.sim;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ArmConfig;
import frc.robot.constants.ArmConstants;

public class ArmSim implements AutoCloseable{

    TalonFXSimState armPivot;

    DCMotor armGearbox = DCMotor.getKrakenX60Foc(1);
    
    SingleJointedArmSim arm;

    double motRotations;

    Mechanism2d arm_vis;
    MechanismRoot2d arm_root;
    MechanismLigament2d arm_mech;

    public ArmSim(ArmConfig config, TalonFXSimState armPivot) {
        this.armPivot = armPivot;

        this.arm = new SingleJointedArmSim(
            armGearbox, 
            ArmConstants.kArmRotateGearing, 
            ArmConstants.kArmRotateMomentOfInertia, 
            ArmConstants.kArmLength, 
            ArmConstants.kArmMinAngle, 
            ArmConstants.kArmMaxAngle, 
            false, 
            ArmConstants.kArmStartingAngle);

        arm_vis = new Mechanism2d(20, 20);
        arm_root = arm_vis.getRoot("Arm Root", 10, 10);
        arm_mech = arm_root.append(new MechanismLigament2d("Arm", ArmConstants.kArmLength, arm.getAngleRads()));

        SmartDashboard.putData("ARHM", arm_vis);
    }

    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        armPivot.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        arm.setInput(armPivot.getMotorVoltage());
        arm.update(0.02);

        //Update sensor positions.

        //rotations = (radians / 2*pi) * gear ratio
        motRotations = (arm.getAngleRads() * ArmConstants.kArmRotateGearing);

        armPivot.setRawRotorPosition(motRotations * ArmConstants.kArmRotateGearing);
        armPivot.setRotorVelocity(motRotations * ArmConstants.kArmRotateGearing);

        arm_mech.setAngle(arm.getAngleRads());     
    }

    @Override
    public void close() {
        arm_vis.close();
    }

}
