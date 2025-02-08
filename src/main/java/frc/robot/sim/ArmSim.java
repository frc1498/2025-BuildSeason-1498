package frc.robot.sim;

import com.ctre.phoenix6.sim.CANcoderSimState;
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
    CANcoderSimState armEncoder;

    DCMotor armGearbox = DCMotor.getKrakenX60Foc(1);
    
    SingleJointedArmSim arm;

    double motRotations;
    double outputDegrees;
    double outputDegreesPerSec;
    double outputRotations;
    double outputRotationsPerSec;
    double inputRotations;
    double inputRotationsPerSec;

    Mechanism2d arm_vis;
    MechanismRoot2d arm_root;
    MechanismLigament2d arm_mech;

    public ArmSim(ArmConfig config, TalonFXSimState armPivot, CANcoderSimState armEncoder) {
        this.armPivot = armPivot;
        this.armEncoder = armEncoder;

        this.arm = new SingleJointedArmSim(
            armGearbox, 
            ArmConstants.kArmRotateGearing, 
            ArmConstants.kArmRotateMomentOfInertia, 
            ArmConstants.kArmLength, 
            ArmConstants.kArmMinAngle, 
            ArmConstants.kArmMaxAngle, 
            true, 
            ArmConstants.kArmStartingAngle);

        arm_vis = new Mechanism2d(20, 20);
        arm_root = arm_vis.getRoot("Arm Root", 10, 10);
        //Multiply arm length by 5 to make it more visible.
        arm_mech = arm_root.append(new MechanismLigament2d("Arm", ArmConstants.kArmLength * 5, arm.getAngleRads()));

        SmartDashboard.putData("ARHM", arm_vis);
    }

    public void simulationPeriodic() {
        //Set motor and sensor voltage.
        armPivot.setSupplyVoltage(12);
        armEncoder.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        arm.setInput(armPivot.getMotorVoltage());
        arm.update(0.02);

        //Update sensor positions.

        //Convert radians to degrees - at the output
        outputDegrees = this.radToDeg(arm.getAngleRads());
        outputDegreesPerSec = this.radToDeg(arm.getVelocityRadPerSec());

        //Convert degrees to rotations - at the output
        outputRotations = this.degToRot(outputDegrees);
        outputRotationsPerSec = this.degToRot(outputDegreesPerSec);

        //Convert output rotations to input rotations
        inputRotations = this.outputRotToInputRot(outputRotations, ArmConstants.kArmRotateGearing);
        inputRotationsPerSec = this.outputRotToInputRot(outputRotationsPerSec, ArmConstants.kArmRotateGearing);

        armPivot.setRawRotorPosition(inputRotations);
        armPivot.setRotorVelocity(inputRotationsPerSec);
        //Assuming 1:1 with the motor position?
        armEncoder.setRawPosition(inputRotations);
        armEncoder.setVelocity(inputRotationsPerSec);

        arm_mech.setAngle(outputDegrees);     
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
        arm_vis.close();
    }

}
