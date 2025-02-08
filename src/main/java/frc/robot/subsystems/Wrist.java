package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.WristConfig;
import frc.robot.constants.WristConstants;
import frc.robot.sim.WristSim;

public class Wrist extends SubsystemBase{
    TalonFX wristRotate;

    CANcoder wristRotateCancoder;

    PositionVoltage rotateControl;

    TalonFX wristSpin;
    VelocityVoltage spinControl;
    
    TalonFXSimState wristRotateSim;
    CANcoderSimState wristEncoderSim;
    TalonFXSimState wristRollerSim;

    WristConfig config;
    WristSim sim;

    double wristDesiredPosition;

   
    public Wrist(WristConfig config) {
        
        this.config = config;

        wristRotate = new TalonFX(config.kRotateCANID, "canivore");
        wristRotateCancoder = new CANcoder(config.kEncoderCANID,"canivore");
        rotateControl = new PositionVoltage(WristConstants.kCoralStow);

        wristSpin = new TalonFX(config.kSpinCANID, "canivore");
        spinControl = new VelocityVoltage(WristConstants.kCoralStop);

        this.configureMechanism(wristSpin, config.wristSpinConfig);  //Fill in framework
        this.configureMechanism(wristRotate, config.wristRotateConfig);  //Fill in framework
        this.configureCancoder(wristRotateCancoder, config.wristRotateCANcoderConfig);  //Fill in framework
      
        wristRotateSim = wristRotate.getSimState();
        wristEncoderSim = wristRotateCancoder.getSimState();
        wristRollerSim = wristSpin.getSimState();
        sim = new WristSim(config, wristRotateSim, wristEncoderSim, wristRollerSim);
        
        SmartDashboard.putData("Wrist", this);
    }
    
    public void configureCancoder(CANcoder coralIntakeRotate, CANcoderConfiguration config){       
        //Start Configuring Climber Motor
        StatusCode coralIntakeRotateStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            coralIntakeRotateStatus = coralIntakeRotate.getConfigurator().apply(config);
            if (coralIntakeRotateStatus.isOK()) break;
        }
        if (!coralIntakeRotateStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + coralIntakeRotateStatus.toString());
        }
    }

    public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config){     
        //Start Configuring Climber Motor
        StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            mechanismStatus = mechanism.getConfigurator().apply(config);
            if (mechanismStatus.isOK()) break;
        }
        if (!mechanismStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
        }
    }

    //=============================================================
    //====================Private Methods==========================
    //=============================================================
    private void wristDriveToPosition(double position) {
        this.wristDesiredPosition = position;
            wristRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean isWristAtPosition(double position) {
            return ((position-WristConstants.kDeadband) <= getWristPosition()) && ((position+WristConstants.kDeadband) >= getWristPosition());
    }

    private double getDesiredPostion() {
        return wristDesiredPosition;
    }

    private double getWristPosition(){
            return wristRotate.getPosition().getValueAsDouble();       
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
        builder.addDoubleProperty("Desired Position", this::getDesiredPostion, null);
    }  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
