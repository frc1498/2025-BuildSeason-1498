package frc.robot.sim;

import com.ctre.phoenix6.sim.CANcoderSimState;
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
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;

public class WristSim implements AutoCloseable{

    TalonFXSimState wristDrive;
    CANcoderSimState wristEncoder;
    TalonFXSimState wristSpinny;

    DCMotor wristGearbox = DCMotor.getKrakenX60Foc(1);
    DCMotor wristRoll = DCMotor.getKrakenX60Foc(1);
    
    SingleJointedArmSim wrist;
    FlywheelSim wristRoller;

    double motRotations;
    double motSpeed;
    private double rollerRotationsPerSec;
    private double rollerPosition;
    private double outputDegrees;
    private double outputDegreesPerSec;
    private double outputRotations;
    private double outputRotationsPerSec;
    private double inputRotations;
    private double inputRotationsPerSec;

    Mechanism2d wrist_vis;
    MechanismRoot2d wrist_root;
    MechanismLigament2d wrist_rotate_mech;
    MechanismLigament2d wrist_roller_mech;

    public WristSim(WristConfig config, TalonFXSimState wristDrive, CANcoderSimState wristEncoder, TalonFXSimState wristRolly) {
        this.wristDrive = wristDrive;
        this.wristEncoder = wristEncoder;
        this.wristSpinny = wristRolly;

        rollerPosition = 0;

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
        wristEncoder.setSupplyVoltage(12);
        wristSpinny.setSupplyVoltage(12);
        
        //Run the simulation and update it.
        wrist.setInput(wristDrive.getMotorVoltage());
        wristRoller.setInput(wristSpinny.getMotorVoltage());
        wrist.update(0.02);
        wristRoller.update(0.02);

        //Update sensor positions.

        //Convert radians to degrees - at the output
        outputDegrees = this.radToDeg(wrist.getAngleRads());
        outputDegreesPerSec = this.radToDeg(wrist.getVelocityRadPerSec());

        //Convert degrees to rotations - at the output
        outputRotations = this.degToRot(outputDegrees);
        outputRotationsPerSec = this.degToRot(outputDegreesPerSec);

        //Convert output rotations to input rotations
        inputRotations = this.outputRotToInputRot(outputRotations, WristConstants.kWristRotateGearing);
        inputRotationsPerSec = this.outputRotToInputRot(outputRotationsPerSec, WristConstants.kWristRotateGearing);


        rollerRotationsPerSec = wristRoller.getAngularVelocityRPM() / 60;
        //Assuming an update every 20ms, find the distance traveled over 20ms and add it to the position.
        rollerPosition += rollerRotationsPerSec * 0.02;

        wristDrive.setRawRotorPosition(inputRotations);
        wristDrive.setRotorVelocity(inputRotationsPerSec);
        //Assuming 1-to-1 with the motor?
        wristEncoder.setRawPosition(inputRotations);
        wristEncoder.setVelocity(inputRotationsPerSec);

        wristSpinny.setRotorVelocity(rollerRotationsPerSec);

        wrist_rotate_mech.setAngle(this.radToAngle(wrist.getAngleRads()));
        wrist_roller_mech.setAngle(rollerPosition);    
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
        wrist_vis.close();
    }
    
    private double radToAngle(double radians) {
        return (radians / (2 * Math.PI)) * 360;
    }
}
