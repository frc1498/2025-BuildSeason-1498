package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.WristConstants;
import frc.robot.sim.ElleySim;
import frc.robot.sim.WristSim;

public class Wrist extends SubsystemBase{
    TalonFX wristRotate;
    CANcoder wristRotateEncoder;
    PositionVoltage rotateControl;

    PositionVoltage posControl;
    
    TalonFXSimState wristDriveFrontSim = wristRotate.getSimState();

    WristConfig config;
    WristSim sim;

   
    public Wrist(WristConfig config) {
        //Constructor
        wristRotate = new TalonFX(WristConstants.kWristRotateCANID, "canivore");
        wristRotateEncoder = new CANcoder(WristConstants.kWristEncoderCANID,"canivore");
        //rotateControl = new PositionVoltage(WristConstants.kCoralStow);

        wristRotate.getConfigurator().apply(config.WristConfig);
        this.config = config;
        //sim = new WristSim(config, wristDriveFrontSim);
        posControl = new PositionVoltage(0);
    }
    
    //=============================================================
    //====================Private Methods==========================
    //=============================================================
    private void wristDriveToPosition(double position) {
            wristRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean isWristAtPosition(double position) {
            return ((position-WristConstants.kDeadband) <= getWristPosition()) && ((position+WristConstants.kDeadband) >= getWristPosition());
    }

    private double getWristPosition(){
            return wristRotate.getPosition().getValueAsDouble();       
    }

    //===================================================
    //=====================Public Methods===============
    //===================================================    
    public void configureWristRotate(TalonFX wristRotate){
        //Start Configuring Wrist Motor
        TalonFXConfiguration wristRotateConfig = new TalonFXConfiguration();

        wristRotateConfig.MotorOutput.Inverted = WristConstants.kRotateDirection;
        wristRotateConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristRotateConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.kRotateSupplyCurrentLimit;
        wristRotateConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = WristConstants.kRotateVoltageClosedLoopRampPeriod;
        wristRotateConfig.Voltage.PeakForwardVoltage = WristConstants.kRotateMaxForwardVoltage;
        wristRotateConfig.Voltage.PeakReverseVoltage = WristConstants.kRotateMaxReverseVoltage;
        wristRotateConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //Configure Gains and Motion Magic Items
        Slot0Configs slot0 = wristRotateConfig.Slot0;
        slot0.kP = WristConstants.kRotateProportional;
        slot0.kI = WristConstants.kRotateIntegral;
        slot0.kD = WristConstants.kRotateDerivative;
        //slot0.GravityType = ;  //We probably don't need this for the wrist
        slot0.kV = WristConstants.kRotateVelocityFeedFoward;
        //slot0.kS = WristConstants.kClimberStaticFeedFoward;  //Probably don't need this for the wrist?

        //Setting the config option that allows playing music on the motor during disabled.
        wristRotateConfig.Audio.AllowMusicDurDisable = true;
 
        StatusCode wristRotateStatus = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            wristRotateStatus = wristRotate.getConfigurator().apply(wristRotateConfig);
            if (wristRotateStatus.isOK()) break;
        }
        if (!wristRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + wristRotateStatus.toString());
        }
        //m_LeftClimb.setPosition(0);
    }

    public Command wristCoralStow() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralStow);}
        ).until(this.isWristCoralStow);
    }

    public Command wristCoralLoadFloor() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralLoadFloor);}
        ).until(this.isWristCoralLoadFloor);
    }

    public Command wristCoralLoadHuman() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralLoadHuman);}
        ).until(this.isWristCoralLoadHuman);
    }

    public Command wristCoralL1() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL1);}
        ).until(this.isWristCoralL1);
    }

    public Command wristCoralL2() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL2);}
        ).until(this.isWristCoralL2);
    }
    
    public Command wristCoralL3() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL3);}
        ).until(this.isWristCoralL3);
    }

    public Command wristCoralL4() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL4);}
        ).until(this.isWristCoralL4);
    }

    public Command wristAlgaeStow() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeStow);}
        ).until(this.isWristAlgaeStow);
    }

    public Command wristAlgaeLoadFloor() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeLoadFloor);}
        ).until(this.isWristAlgaeLoadFloor);
    }

    public Command wristAlgaeL2() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeL2);}
        ).until(this.isWristAlgaeL2);
    }
    
    public Command wristAlgaeL3() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeL3);}
        ).until(this.isWristAlgaeL3);
    }
    
    public Command wristAlgaeBarge() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeBarge);}
        ).until(this.isWristAlgaeBarge);
    }

    public Command wristAlgaeProcessor() {
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeProcessor);}
        ).until(this.isWristAlgaeProcessor);
    }

    //========Triggers for Wrist Coral=========
    public final Trigger isWristCoralStow = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralStow);});
    public final Trigger isWristCoralLoadFloor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadFloor);});
    public final Trigger isWristCoralLoadHuman = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadHuman);});
    public final Trigger isWristCoralL1 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL1);});
    public final Trigger isWristCoralL2 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL2);});
    public final Trigger isWristCoralL3 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL3);});
    public final Trigger isWristCoralL4 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL4);});

    //=========Triggers for Wrist Algae========
    public final Trigger isWristAlgaeStow = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeStow);});
    public final Trigger isWristAlgaeLoadFloor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeLoadFloor);});
    public final Trigger isWristAlgaeL2 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeL2);});
    public final Trigger isWristAlgaeL3 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeL3);});
    public final Trigger isWristAlgaeBarge = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeBarge);});
    public final Trigger isWristAlgaeProcessor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeProcessor);});

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
    }
}
