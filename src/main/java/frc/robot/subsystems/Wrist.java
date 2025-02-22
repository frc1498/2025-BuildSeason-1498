package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.WristConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;
import frc.robot.sim.WristSim;

public class Wrist extends SubsystemBase{
    public TalonFX wristRotate;
    public DutyCycleOut rotateDutyCycleControl;

    CANcoder wristRotateCancoder;

    CANrange wristReefDistance;

    MotionMagicVoltage rotateControl;


    public TalonFX wristSpin;
    public VelocityVoltage spinControl;
    
    TalonFXSimState wristRotateSim;
    CANcoderSimState wristEncoderSim;
    TalonFXSimState wristRollerSim;

    WristConfig config;
    WristSim sim;

    double wristDesiredPosition;


    DigitalInput m_BeamBreakGripperFrontDigital;
    DigitalInput m_BeamBreakGripperRearDigital;

   
    Debouncer m_Debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    public Wrist(WristConfig config) {
        //Constructor
        this.config = config;

        wristRotate = new TalonFX(config.kRotateCANID, "canivore");
        wristRotateCancoder = new CANcoder(config.kEncoderCANID,"canivore");
        rotateControl = new MotionMagicVoltage(WristConstants.kCoralStow);
        wristReefDistance = new CANrange(config.kRangeCANID, "canivore");

        m_BeamBreakGripperFrontDigital = new DigitalInput(config.kBeamBreakGripperFront);
        m_BeamBreakGripperRearDigital = new DigitalInput(config.kBeamBreakGripperRear);

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

    //=====================================================================
    //=====================Motor Configure=================================
    //=====================================================================

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

        //    System.out.println("=============Private Wrist wristDriveToPosition===============");
        

        this.wristDesiredPosition = position;
        if (Constants.kWristRotateMotorEnabled == true) {
            wristRotate.setControl(rotateControl.withPosition(position));
        }
    }

    private void wristSpin(double speed) {

        //    System.out.println("=============Private Wrist wristSpin===============");
        
        if (Constants.kWristSpinMotorEnabled == true) {
            wristSpin.setControl(spinControl.withVelocity(speed));
        }
    }

    private boolean isWristAtPosition(double position) {

        //    System.out.println("=============Private Wrist isWristAtPosition===============");


            return ((position-WristConstants.kDeadband) <= getWristPosition()) && ((position+WristConstants.kDeadband) >= getWristPosition());
    }

    private double getDesiredPosition() {

            //System.out.println("=============Private Wrist getDesiredPosition===============");
        

        return wristDesiredPosition;
    }

    private double getWristPosition(){

           //System.out.println("=============Private Wrist wristDriveToPosition===============");
        
            return wristRotate.getPosition().getValueAsDouble();       
    }

    private double getReefDistance() {
        return wristReefDistance.getDistance().getValueAsDouble();
    }

    private void positionCoral(){

         //   System.out.println("=============Private Wrist wristDriveToPosition===============");
        

        if (isPartForwardGripper()){ //front beam break is made
            wristSpin.set(WristConstants.kCoralSlowBackward); //move coral backward slowly
        } else if (isPartRearwardGripper()){ //rear beam break is made
            wristSpin.set(WristConstants.kCoralSlowForward);//move coral forward slowly
        } else if (isPartForwardGripper() && isPartRearwardGripper()){ // both beam breaks are made
            wristSpin.set(WristConstants.kCoralStop); //stop motor
        }
    }

    private boolean isPartForwardGripper() {
        //If either beam break is made, part is in gripper
        if (!m_BeamBreakGripperFrontDigital.get())
        {
            return true;
        } else {
            return false;
        }
    }

    private boolean isPartRearwardGripper() {
        if (!m_BeamBreakGripperRearDigital.get())
        {
            return true;
        } else {      
            return false;
        }
    }

    private boolean isPartGripper() {

        //If neither beam break is made, part is in gripper
        if (!m_Debouncer.calculate(m_BeamBreakGripperFrontDigital.get()) && !m_Debouncer.calculate(m_BeamBreakGripperRearDigital.get()))
        {
            return true;
        } else {
            return false;
        }
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    //=============================================================
    //========================Commands=============================
    //=============================================================
    public Command wristFrontSafe() {

        //    System.out.println("=============Command wrist wristFrontSafe===============");
        

        return run(
            () -> {this.wristDriveToPosition(WristConstants.kFrontSafe);}
        ).until(this.isWristFrontSafe);
    }
    
    public Command wristRearSafe() {

        //    System.out.println("=============Command wrist wristRearSafe===============");
        

        return run(
            () -> {this.wristDriveToPosition(WristConstants.kRearSafe);}
        ).until(this.isWristRearSafe);
    }

    public Command wristCoralStow() {

        //    System.out.println("=============Command wrist wristCoralStow===============");
        

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

        //   System.out.println("=============Command wrist wristCoralL1===============");
        
        
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL1);}
        ).until(this.isWristCoralL1)
        .withName("wristCoralL1");
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

        //    System.out.println("=============Command wrist wristCoralL4===============");
        

        return run(
            () -> {this.wristDriveToPosition(WristConstants.kCoralL4);}
        ).until(this.isWristCoralL4);
    }

    public Command wristAlgaeStow() {

         //   System.out.println("=============Command wrist wristAlgaeStow===============");
        

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

        //    System.out.println("=============Command wrist wristAlgaeL2===============");
        
        
        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeL2);}
        ).until(this.isWristAlgaeL2);
    }
    
    public Command wristAlgaeL3() {

        //    System.out.println("=============Command wrist wristAlgaeL3===============");
        

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

         //   System.out.println("=============Command wrist wristAlgaeProcesor===============");
        

        return run(
            () -> {this.wristDriveToPosition(WristConstants.kAlgaeProcessor);}
        ).until(this.isWristAlgaeProcessor);
    }

    public Command positionCoralInGripper(){

        //    System.out.println("=============Command wrist positionCoralInGripper===============");
        

        return run(
            () -> {this.positionCoral();}
        ).until(this.isWristAlgaeProcessor);
    }

    public Command suck() {

         //   System.out.println("=============Command wrist suck===============");
        

        return run(
            () -> {this.wristSpin(WristConstants.kCoralSuck);}
        );
    }

    public Command spit() {

        //    System.out.println("=============Command wrist spit===============");
        

        return run(
            () -> {this.wristSpin(WristConstants.kCoralSpit);}
        );
    }

    public Command stop() {

        //    System.out.println("=============Command wrist stop spinning===============");
        

        return runOnce(
            () -> {this.wristSpin(WristConstants.kCoralStop);}
        );
    }

    public Command toWristPosition(double position) {

        //    System.out.println("=============Command wrist toWristPosition===============");
        

        return run(
            () -> {this.wristDriveToPosition(position);}
        );
    }

    //===============================================
    //=================Suppliers=====================
    //===============================================

    public DoubleSupplier getWristRotation() {
        return this::getWristPosition; 
    }

    //=======================================================
    //=================Triggers for Wrist Coral==============
    //=======================================================
    public final Trigger isPartRearwardGripper = new Trigger(() -> {return this.isPartRearwardGripper();});
    public final Trigger isPartForwardGripper = new Trigger(() -> {return this.isPartForwardGripper();});
    public final Trigger isPartInGripper = new Trigger(() -> {return this.isPartGripper();});
    public final Trigger isWristCoralStow = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralStow);});
    public final Trigger isWristCoralLoadFloor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadFloor);});
    public final Trigger isWristCoralLoadHuman = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadHuman);});
    public final Trigger isWristCoralL1 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL1);});
    public final Trigger isWristCoralL2 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL2);});
    public final Trigger isWristCoralL3 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL3);});
    public final Trigger isWristCoralL4 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL4);});

    //========================================================
    //===================Triggers for Wrist Algae=============
    //========================================================
    public final Trigger isWristAlgaeStow = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeStow);});
    public final Trigger isWristAlgaeLoadFloor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeLoadFloor);});
    public final Trigger isWristAlgaeL2 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeL2);});
    public final Trigger isWristAlgaeL3 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeL3);});
    public final Trigger isWristAlgaeBarge = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeBarge);});
    public final Trigger isWristAlgaeProcessor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kAlgaeProcessor);});

    //=====================================================
    //===============Triggers General======================
    //=====================================================
    public final Trigger isWristFrontSafe = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kFrontSafe);});
    public final Trigger isWristRearSafe = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kRearSafe);});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Desired Position", this::getDesiredPosition, null);
        builder.addBooleanProperty("Front Gripper Beam Break", () -> {return isPartForwardGripper();}, null);
        builder.addBooleanProperty("Rear Gripper Beam Break", () -> {return isPartRearwardGripper();}, null);
        builder.addDoubleProperty("CANrange Reef Distance", this::getReefDistance, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addBooleanProperty("Is the wrist at CoralLoadFloor", isWristCoralLoadFloor,null);
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
