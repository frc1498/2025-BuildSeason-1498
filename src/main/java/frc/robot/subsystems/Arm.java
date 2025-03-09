package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ArmConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.sim.ArmSim;

public class Arm extends SubsystemBase{
    //Declare Variables
    public TalonFX armRotate;
    MotionMagicVoltage rotateControl;
    public DutyCycleOut rotateDutyCycleControl;

    CANcoder armRotateEncoder;

    TalonFXSimState armSim;
    CANcoderSimState armEncoderSim;

    ArmSim sim;

    private double desiredPosition;

    enum armState {
        FRONT,
        REAR,
        MIDDLE,
        NONE
    }

    public Arm(ArmConfig config) {
        //Constructor - only runs once

        //Instantiate
        armRotate = new TalonFX(config.kArmRotateCANID, "canivore");
        armRotateEncoder = new CANcoder(config.kEncoderCANID,"canivore");
        rotateControl = new MotionMagicVoltage(ArmConstants.kCoralStow);

        //Fill In Instatiations
        this.configureMechanism(armRotate, config.armRotateConfig);
        this.configureCancoder(armRotateEncoder, config.armRotateCANcoderConfig);
  
        armSim = armRotate.getSimState();
        armEncoderSim = armRotateEncoder.getSimState();

        sim = new ArmSim(config, armSim, armEncoderSim);

        SmartDashboard.putData("Arm", this);
    }

    public void configureCancoder(CANcoder coralIntakeRotate, CANcoderConfiguration config){       

//=====================================================
//===================Configuration=====================
//=====================================================      

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

    //===================================================
    //=====================Private Methods===============
    //===================================================
    private void armDriveToPosition(double position) {
        this.desiredPosition = position;
        if (Constants.kArmRotateMotorEnabled == true) {
            armRotate.setControl(rotateControl.withPosition(position));
        }
    }

    private boolean isArmAtPosition(double position) {
            return ((position-ArmConstants.kDeadband) <= GetArmPosition()) && ((position+ArmConstants.kDeadband) >= GetArmPosition());
    }

    private double getDesiredArmPosition() {
        return this.desiredPosition;
    }

    private double GetArmPosition(){
        return armRotate.getPosition().getValueAsDouble();
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    private boolean IsArmInFrontOfIntake() {
        return (armRotate.getPosition().getValueAsDouble() < ArmConstants.kIntakeSafe);
    }

    private boolean IsArmAboveIntake45() {
        return (armRotate.getPosition().getValueAsDouble() > (ArmConstants.kArmAboveIntake45-0.05));
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
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeL2);}
        ).until(this.isArmAlgaeL2);
    }

    public Command armAlgaeL3() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeL3);}
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

    public DoubleSupplier getArmRotation() {
        return this::GetArmPosition; 
    }

    public Command toArmPosition(double position) {
        return run(
            () -> {this.armDriveToPosition(position);}
        );
    }

    public Command armAboveIntake45L3() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL3);}
        ).until(this.isArmAboveIntake45);
    }

    public Command armClearClimb() {
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kClearClimb);}
        ).until(this.isArmClearClimb);
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
    public final Trigger isArmAlgaeL2 = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeL2);});
    public final Trigger isArmAlgaeL3 = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeL3);});
    public final Trigger isArmAlgaeBarge = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeBarge);});
    public final Trigger isArmAlgaeProcessor = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kAlgaeProcessor);});

    //===========================================
    //===============General Triggers============
    //===========================================
    public final Trigger isArmFrontSafe = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kFrontSafe);});
    public final Trigger isArmRearSafe = new Trigger(() -> {return this.isArmAtPosition(ArmConstants.kRearSafe);});
    public final Trigger isArmClearClimb = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kClearClimb);});
    public final Trigger isArmInFrontOfIntake = new Trigger(() ->{return this.IsArmInFrontOfIntake();});
    public final Trigger isArmAboveIntake45 = new Trigger(() ->{return this.IsArmAboveIntake45();});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Desired Position", this::getDesiredArmPosition, null);
        builder.addDoubleProperty("Current Position", this::GetArmPosition, null);
        builder.addBooleanProperty("Arm L1 Position", isArmCoralL1, null);
        builder.addBooleanProperty("Arm Human Coral Load", isArmCoralLoadHuman, null);
        builder.addBooleanProperty("Arm CoralLoadFloor", isArmCoralLoadFloor,null);
        builder.addBooleanProperty("Arm Front of Intake", isArmInFrontOfIntake,null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //Cache the previous arm state, then update the current arm state and the desired arm state.
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
