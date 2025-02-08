package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ArmConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.sim.ArmSim;

public class Arm extends SubsystemBase{
    //Declare Variables
    TalonFX armRotate;
    PositionVoltage rotateControl;

    CANcoder armRotateEncoder;

    TalonFXSimState armSim;

    ArmSim sim;

    public Arm(ArmConfig config) {
        //Constructor - only runs once

        //Instantiate
        armRotate = new TalonFX(config.kArmRotateCANID, "canivore");
        armRotateEncoder = new CANcoder(config.kEncoderCANID,"canivore");
        rotateControl = new PositionVoltage(ArmConstants.kCoralStow);

        //Fill In Instatiations
        this.configureMechanism(armRotate);
        this.configureCancoder(armRotateEncoder);
  
        armSim = armRotate.getSimState();

        sim = new ArmSim(config, armSim);
    }

//=====================================================
//===================Configuration=====================
//=====================================================
    public void configureCancoder(CANcoder coralIntakeRotate){       
        //Start Configuring Climber Motor
        CANcoderConfiguration coralIntakeRotateConfig = new CANcoderConfiguration();
        StatusCode coralIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            coralIntakeRotateStatus = coralIntakeRotate.getConfigurator().apply(coralIntakeRotateConfig);
            if (coralIntakeRotateStatus.isOK()) break;
        }
        if (!coralIntakeRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + coralIntakeRotateStatus.toString());
        }
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

    //===================================================
    //=====================Private Methods===============
    //===================================================
    private void armDriveToPosition(double position) {
            armRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean isArmAtPosition(double position) {
            return ((position-ArmConstants.kDeadband) <= GetArmPosition()) && ((position+ArmConstants.kDeadband) >= GetArmPosition());
    }

    private double GetArmPosition(){
            return armRotate.getPosition().getValueAsDouble();
    }

    //===================================================
    //=====================Public Commands===============
    //===================================================    
    public Command armCoralStow() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralStow);}
        ).until(this.isArmCoralStow);
    }

    public Command armCoralLoadFloor() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralLoadFloor);}
        ).until(this.isArmCoralLoadFloor);
    }

    public Command armCoralLoadHuman() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralLoadHuman);}
        ).until(this.isArmCoralLoadHuman);
    }

    public Command armCoralL1() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL1);}
        ).until(this.isArmCoralL1);
    }

    public Command armCoralL2() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL2);}
        ).until(this.isArmCoralL2);
    }

    public Command armCoralL3() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL3);}
        ).until(this.isArmCoralL3);
    }

    public Command armCoralL4() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL4);}
        ).until(this.isArmCoralL4);
    }

    public Command armAlgaeStow() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeStow);}
        ).until(this.isArmAlgaeStow);
    }

    public Command armAlgaeLoadFloor() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeLoadFloor);}
        ).until(this.isArmAlgaeLoadFloor);
    }

    public Command armAlgaeL2() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeLoadL2);}
        ).until(this.isArmAlgaeL2);
    }

    public Command armAlgaeL3() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeLoadL3);}
        ).until(this.isArmAlgaeL3);
    }

    public Command armAlgaeBarge() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeBarge);}
        ).until(this.isArmAlgaeBarge);
    }

    public Command armAlgaeProcessor() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeProcessor);}
        ).until(this.isArmAlgaeProcessor);
    }
    //============================================
    //===============Coral Triggers===============
    //============================================
    public Trigger isArmCoralStow = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralStow);});
    public Trigger isArmCoralLoadFloor = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralLoadFloor);});
    public Trigger isArmCoralLoadHuman = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralLoadHuman);});
    public Trigger isArmCoralL1 = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralL1);});
    public Trigger isArmCoralL2 = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralL2);});
    public Trigger isArmCoralL3 = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralL3);});
    public Trigger isArmCoralL4 = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralL4);});

    //===========================================
    //===============Algae Triggers==============
    //===========================================
    public final Trigger isArmAlgaeStow = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeStow);});
    public final Trigger isArmAlgaeLoadFloor = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeLoadFloor);});
    public final Trigger isArmAlgaeL2 = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeLoadL2);});
    public final Trigger isArmAlgaeL3 = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeLoadL3);});
    public final Trigger isArmAlgaeBarge = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeBarge);});
    public final Trigger isArmAlgaeProcessor = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeProcessor);});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
