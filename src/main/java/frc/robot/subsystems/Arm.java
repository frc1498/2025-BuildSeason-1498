package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private armState currentArmState;
    private armState previousArmState;
    private armState desiredArmState;

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

        previousArmState = armState.NONE;
        currentArmState = armState.NONE;
        desiredArmState = armState.NONE;

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

        //    System.out.println("=========================Private armDriveToPosition=====================");
        //    System.out.println("armDriveToPosition:"+this.desiredPosition);

        
        if (Constants.kArmRotateMotorEnabled == true) {
            armRotate.setControl(rotateControl.withPosition(position));
        }
    }

    private boolean isArmAtPosition(double position) {

        //    System.out.println("=========================Private isArmAtPosition=====================");
        //    System.out.println("LowerBound:"+(position+ArmConstants.kDeadband));
        //    System.out.println("Actual:"+position);
        //    System.out.println("UpperBound:"+(position-ArmConstants.kDeadband));

            return ((position-ArmConstants.kDeadband) <= GetArmPosition()) && ((position+ArmConstants.kDeadband) >= GetArmPosition());
    }

    private double getDesiredArmPosition() {
 
        //    System.out.println("=========================Private Arm getDesiredPosition=====================");
        //    System.out.println("Desired Position:"+this.desiredPosition);

        return this.desiredPosition;
    }

    private double GetArmPosition(){
        
        // System.out.println("========================= Private Arm getArmPosition=====================");
        // System.out.println("Arm Position:"+armRotate.getPosition().getValueAsDouble());


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

    private armState getCurrentArmState() {
        return currentArmState;
    }

    private armState getDesiredArmState() {
        return desiredArmState;
    }

    private armState getPreviousArmState() {
        return previousArmState;
    }

    private armState checkArmState(double currentArmPosition) {
        if (currentArmPosition < ArmConstants.kFrontSafe) {
            return armState.FRONT;
        }
        else if (currentArmPosition > ArmConstants.kRearSafe) {
            return armState.REAR;
        }
        else if ((currentArmPosition > ArmConstants.kFrontSafe) && (currentArmPosition < ArmConstants.kRearSafe)) {
            return armState.MIDDLE;
        }
        else {
            return armState.NONE;
        }
    }
    
    private boolean IsArmIntakeSafe() {

        return (armRotate.getPosition().getValueAsDouble() < ArmConstants.kIntakeSafe);

    }

    //===================================================
    //=====================Public Commands===============
    //===================================================    

    public Command armFrontSafe() {

        //    System.out.println("=============Command Arm armFrontSafe===============");


        return run(    
            () -> {this.armDriveToPosition(ArmConstants.kFrontSafe);}
        ).until(this.isArmFrontSafe);
    }
    
    public Command armRearSafe() {

        //    System.out.println("=============Command Arm armRearSafe===============");


        return run(
            () -> {this.armDriveToPosition(ArmConstants.kRearSafe);}
        ).until(this.isArmRearSafe);
    }

    public Command armCoralStow() {

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralStow);}
        ).until(this.isArmCoralStow);
    }

    public Command armCoralLoadFloor() {

        //System.out.println("=============Command Arm armCoralLoadFloor===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralLoadFloor);}
        ).until(this.isArmCoralLoadFloor);
    }

    public Command armCoralLoadFloorBetter() {

        //System.out.println("=============Command Arm armCoralLoadFloor===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralLoadFloorBetter);}
        ).until(this.isArmCoralLoadFloorBetter);
    }



    public Command armCoralLoadHuman() {
        
        //System.out.println("=============Command Arm armCoralLoadHuman===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralLoadHuman);}
        ).until(this.isArmCoralLoadHuman);
    }

    public Command armCoralL1() {

        //System.out.println("=============Command Arm armCoralL1===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL1);}
        ).until(this.isArmCoralL1);
    }

    public Command armCoralL2() {

        //    System.out.println("=============Command Arm armCoralL2===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kCoralL2);}
        ).until(this.isArmCoralL2);
    }

    public Command armCoralL3() {

        //System.out.println("=============Command Arm armCoralL3===============");


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

         //   System.out.println("=============Command Arm armAlgaeLoadFloor===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeLoadFloor);}
        ).until(this.isArmAlgaeLoadFloor);
    }

    public Command armAlgaeL2() {

        //    System.out.println("=============Command Arm armAlgaeL2===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeL2);}
        ).until(this.isArmAlgaeL2);
    }

    public Command armAlgaeL3() {

        //    System.out.println("=============Command Arm armAlgaeL3===============");


        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeL3);}
        ).until(this.isArmAlgaeL3);
    }

    public Command armAlgaeBarge() {

        //    System.out.println("=============Command Arm armAlgaeBarge===============");

        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeBarge);}
        ).until(this.isArmAlgaeBarge);
    }

    public Command armAlgaeProcessor() {
        
        //System.out.println("=============Command Arm armAlgaeProcessor===============");
        
        return run(
            () -> {this.armDriveToPosition(ArmConstants.kAlgaeProcessor);}
        ).until(this.isArmAlgaeProcessor);
    }

    public DoubleSupplier getArmRotation() {

        //   System.out.println("=============Command Arm getArmRotation===============");


        return this::GetArmPosition; 
    }

    public Command toArmPosition(double position) {
         //   System.out.println("=============Command Arm toArmPosition===============");

        return run(
            () -> {this.armDriveToPosition(position);}
        );
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
    public Trigger isArmCoralLoadFloorBetter = new Trigger(() ->{return this.isArmAtPosition(ArmConstants.kCoralLoadFloorBetter);});

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
    public final Trigger isArmFront = new Trigger(() -> {return this.currentArmState == armState.FRONT;});
    public final Trigger isArmMiddle = new Trigger(() -> {return this.currentArmState == armState.MIDDLE;});
    public final Trigger isArmRear = new Trigger(() -> {return this.currentArmState == armState.REAR;});

    public final Trigger isArmIntakeSafe = new Trigger(() ->{return this.IsArmIntakeSafe();});


    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Desired Position", this::getDesiredArmPosition, null);
        builder.addDoubleProperty("Current Position", this::GetArmPosition, null);
        builder.addBooleanProperty("Is Arm at L1 Position", isArmCoralL1, null);
        builder.addBooleanProperty("Is Arm at Human Coral Load", isArmCoralLoadHuman, null);
        builder.addBooleanProperty("Is the Arm at CoralLoadFloor", isArmCoralLoadFloor,null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addStringProperty("Previous Arm State", () -> {return this.getPreviousArmState().toString();}, null);
        builder.addStringProperty("Current Arm State", () -> {return this.getCurrentArmState().toString();}, null);
        builder.addStringProperty("Desired Arm State", () -> {return this.getDesiredArmState().toString();}, null);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //Cache the previous arm state, then update the current arm state and the desired arm state.
        this.previousArmState = this.currentArmState;
        this.currentArmState = this.checkArmState(this.GetArmPosition());
        this.desiredArmState = this.checkArmState(this.getDesiredArmPosition());
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
