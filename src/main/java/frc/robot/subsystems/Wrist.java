package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.WristConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.WristConstants;
import frc.robot.constants.EndEffectorConstants.endEffectorLocation;
import frc.robot.sim.WristSim;

public class Wrist extends SubsystemBase{
    public TalonFX wristRotate;
    public DutyCycleOut rotateDutyCycleControl;

    CANcoder wristRotateCancoder;

    CANrange wristReefDistance;

    MotionMagicVoltage rotateControl;


    public TalonFX wristSpin;
    public VelocityVoltage spinControl;
    public PositionVoltage spinPositionVoltage;
    
    TalonFXSimState wristRotateSim;
    CANcoderSimState wristEncoderSim;
    TalonFXSimState wristRollerSim;

    WristConfig config;
    WristSim sim;

    double wristDesiredPosition;
    double wristDesiredSpeed;
    
    public boolean range_ok;

    DigitalInput m_BeamBreakGripperFrontDigital;
    DigitalInput m_BeamBreakGripperRearDigital;

    Supplier<endEffectorLocation> endEffectorLocation;
    
    String scoringPosition = "";

    Debouncer m_Debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    public Wrist(WristConfig config, Supplier<endEffectorLocation> endEffectorLocation) {
        //Constructor
        this.config = config;
        this.endEffectorLocation = endEffectorLocation;
        this.wristDesiredSpeed = 0;

        wristRotate = new TalonFX(config.kRotateCANID, "canivore");
        wristRotateCancoder = new CANcoder(config.kEncoderCANID,"canivore");
        rotateControl = new MotionMagicVoltage(WristConstants.kCoralStow);
        wristReefDistance = new CANrange(config.kRangeCANID, "canivore");

        m_BeamBreakGripperFrontDigital = new DigitalInput(config.kBeamBreakGripperFront);
        m_BeamBreakGripperRearDigital = new DigitalInput(config.kBeamBreakGripperRear);

        wristSpin = new TalonFX(config.kSpinCANID, "canivore");
        spinControl = new VelocityVoltage(WristConstants.kCoralStop);
        spinPositionVoltage = new PositionVoltage(0);

        this.configureMechanism(wristSpin, config.wristSpinConfig);  //Fill in framework
        this.configureMechanism(wristRotate, config.wristRotateConfig);  //Fill in framework
        this.configureCancoder(wristRotateCancoder, config.wristRotateCANcoderConfig);  //Fill in framework
        this.configureCANRange(wristReefDistance, config.canRangeConfig); //Fill in framework
      
        wristRotateSim = wristRotate.getSimState();
        wristEncoderSim = wristRotateCancoder.getSimState();
        wristRollerSim = wristSpin.getSimState();
        sim = new WristSim(config, wristRotateSim, wristEncoderSim, wristRollerSim);

        SmartDashboard.putData("Wrist", this);
    }

    public void configureCANRange(CANrange CANRangeSensor, CANrangeConfiguration config ) {

      //Start Configuring CANRange
      StatusCode canRangeRotateStatus = StatusCode.StatusCodeNotInitialized;

      for(int i = 0; i < 5; ++i) {
          canRangeRotateStatus = CANRangeSensor.getConfigurator().apply(config);
          if (canRangeRotateStatus.isOK()) break;
      }
      if (!canRangeRotateStatus.isOK()) {
          System.out.println("Could not configure device. Error: " + canRangeRotateStatus.toString());
      }

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

        this.wristDesiredPosition = position;
        if (Constants.kWristRotateMotorEnabled == true) {
            wristRotate.setControl(rotateControl.withPosition(position));
        }
    }

    private void wristStop() {
        wristSpin.setPosition(0);
        wristSpin.setControl(spinPositionVoltage.withPosition(0).withSlot(1));
        
    }

    private void wristSpin(/*endEffectorLocation endEffectorLocation*/) {
        //this.endEffectorLocation = endEffectorLocation;

        if (Constants.kWristSpinMotorEnabled == true) {
            switch(this.endEffectorLocation.get()){
                case NONE:
                    this.wristDesiredSpeed = WristConstants.kCoralStop;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralStop).withSlot(0));                
                break;
                case CORAL_GROUND_PICKUP:
                    this.wristDesiredSpeed = WristConstants.kCoralSuck;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralSuck).withSlot(0));
                break;
                case CORAL_HUMAN_PICKUP:
                    this.wristDesiredSpeed = WristConstants.kCoralSuck;
                    wristSpin.setControl(spinControl.withVelocity(7).withSlot(0));
                break;
                case CORAL_L1:
                    this.wristDesiredSpeed = WristConstants.kCoralL1Spit;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralL1Spit).withSlot(0));
                break;
                case CORAL_L2:
                    this.wristDesiredSpeed = WristConstants.kCoralL2Spit;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralL2Spit).withSlot(0));
                break;
                case CORAL_L3:
                    this.wristDesiredSpeed = WristConstants.kCoralL3Spit;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralL3Spit).withSlot(0));
                break;
                case CORAL_L4:
                    this.wristDesiredSpeed = WristConstants.kCoralL4Spit;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralL4Spit).withSlot(0));
                break;
                case ALGAE_L2:
                    this.wristDesiredSpeed = WristConstants.kAlgaeRemove;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kAlgaeRemove).withSlot(2));
                break;
                case ALGAE_L3:
                    this.wristDesiredSpeed = WristConstants.kAlgaeRemove;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kAlgaeRemove).withSlot(2));
                break;
                case ALGAE_L4:
                    this.wristDesiredSpeed = WristConstants.kAlgaeSpit;
                    wristSpin.setControl(spinControl.withVelocity(WristConstants.kAlgaeRemove).withSlot(0));
                break;
           }  
        }
    }


    private String getScoringPosition() {
        return this.scoringPosition;
    }

    private boolean isWristAtPosition(double position) {

            return ((position-WristConstants.kDeadband) <= getWristPosition()) && ((position+WristConstants.kDeadband) >= getWristPosition());
    }

    private double getDesiredPosition() {

        return wristDesiredPosition;
    }

    private double getSignalStrength () {

        return wristReefDistance.getSignalStrength().getValueAsDouble();
    }

    private double getWristPosition(){
        
            return wristRotate.getPosition().getValueAsDouble();       
    }

    private double getReefDistance() {
        return wristReefDistance.getDistance().getValueAsDouble();
    }

    private boolean checkRange(String localPosition) {
        if ((localPosition == "CoralL1") || (localPosition == "")){
            range_ok = true;
        } else if (localPosition == "CoralL2") {  
            range_ok=((wristReefDistance.getDistance().getValueAsDouble() < WristConstants.krangeL2) && 
            (wristReefDistance.getSignalStrength().getValueAsDouble() > WristConstants.kRangeL2SignalStrength));
            
            //range_ok = true;
        } else if (localPosition == "CoralL3") {
            range_ok=((wristReefDistance.getDistance().getValueAsDouble() < WristConstants.krangeL3) && 
            (wristReefDistance.getSignalStrength().getValueAsDouble() > WristConstants.kRangeL3SignalStrength));
        } else if (localPosition == "CoralL4") {
            range_ok=((wristReefDistance.getDistance().getValueAsDouble() < WristConstants.krangeL4) && 
            (wristReefDistance.getSignalStrength().getValueAsDouble() > WristConstants.kRangeL4SignalStrength));
        }
        return range_ok;
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

        if (!m_BeamBreakGripperFrontDigital.get() || !m_BeamBreakGripperRearDigital.get())
        {
            return true;
        } else {
            return false;
        }
    }

    private Double getDesiredWristSpeed() {
        return this.wristDesiredSpeed;
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    private String getCurrentCanRangeMode () {
        return scoringPosition;
    }

    public void addToOrchestra(Orchestra robotOrchestra) {
        robotOrchestra.addInstrument(this.wristRotate, 1);
        robotOrchestra.addInstrument(this.wristSpin, 1);
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

    public Command spitAlgae() {

        return run(
            () -> {wristSpin.setControl(spinControl.withVelocity(-100).withSlot(0));});

    }

    public Command wristRearSafe() {

        //    System.out.println("=============Command wrist wristRearSafe===============");
        

        return run(
            () -> {this.wristDriveToPosition(WristConstants.kRearSafe);}
        ).until(this.isWristRearSafe);
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

    /*
    public Command positionCoralInGripper(){

        return run(
            () -> {this.positionCoral();}
        ).until(this.isWristAlgaeProcessor);
    }
    */

    public Command suck(/*Supplier<endEffectorLocation> endEffectorLocation*/) {

        return run(
            () -> {this.wristSpin(/*endEffectorLocation.get()*/);}
        );
    }

    public Command spit(/*Supplier<endEffectorLocation> endEffectorLocation*/) {

        return run(
            () -> {this.wristSpin(/*endEffectorLocation.get()*/);}
        );
    }

    public Command clearWristCoral() {

        return run(
            () -> {wristSpin.setControl(spinControl.withVelocity(WristConstants.kCoralClear).withSlot(0));}
        ).withTimeout(2);
    }

    public Command intialSpit() {
        return run(
            () -> {this.wristSpin();}
        ).withTimeout(0.25);
    }

    public Command stop() {

        return runOnce(
            () -> {this.wristStop(/*endEffectorLocation.NONE*/);}
        );
    }

    public Command toWristPosition(double position) {

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
    public final Trigger isPartForwardGripper = new Trigger(() -> {return this.isPartForwardGripper();}).debounce(0.05);
    public final Trigger isPartInGripper = new Trigger(() -> {return this.isPartGripper();});
    public final Trigger isWristCoralStow = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralStow);});
    public final Trigger isWristCoralLoadFloor = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadFloor);});
    public final Trigger isWristCoralLoadHuman = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadHuman);});
    public final Trigger isWristCoralL1 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL1);});
    public final Trigger isWristCoralL2 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL2);});
    public final Trigger isWristCoralL3 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL3);});
    public final Trigger isWristCoralL4 = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralL4);});

    public final Trigger isCoralWristLoadFloorBetterInitial = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadFloorBetterInitial);});
    public final Trigger isCoralWristLoadFloorBetterFinal = new Trigger(() -> {return this.isWristAtPosition(WristConstants.kCoralLoadFloorBetterFinal);});

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
    public final Trigger isCanRange = new Trigger(() -> {return this.checkRange(this.getScoringPosition());}).debounce(0.05);

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Desired Position", this::getDesiredPosition, null);
        builder.addBooleanProperty("Front Gripper Beam Break", () -> {return isPartForwardGripper();}, null);
        builder.addBooleanProperty("Rear Gripper Beam Break", () -> {return isPartRearwardGripper();}, null);
        builder.addBooleanProperty("Either Beam Break", () -> {return isPartGripper();}, null);
        builder.addDoubleProperty("CANrange Reef Distance", this::getReefDistance, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addBooleanProperty("Is the wrist at CoralLoadFloor", isWristCoralLoadFloor,null);
        builder.addBooleanProperty("Can In Range to Spit", isCanRange, null);
        builder.addStringProperty("Can Range Mode", this::getCurrentCanRangeMode,null);
        builder.addStringProperty("Scoring Position", this::getScoringPosition, null);
        builder.addDoubleProperty("CANrange Signal Strength", this::getSignalStrength, null);
        builder.addDoubleProperty("Desired Wrist Spit Speed", this::getDesiredWristSpeed, null);
    }  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        switch(this.endEffectorLocation.get()) {
            case NONE:
                scoringPosition = "";               
            break;
            case CORAL_GROUND_PICKUP:
                scoringPosition = "";
            break;
            case CORAL_HUMAN_PICKUP:
                scoringPosition = "CoralHumanPickup";
            break;
            case CORAL_L1:
                scoringPosition = "CoralL1";
            break;
            case CORAL_L2:
                scoringPosition = "CoralL2";
            break;
            case CORAL_L3:
                scoringPosition = "CoralL3";
            break;
            case CORAL_L4:
                scoringPosition = "CoralL4";
            break;
            case ALGAE_L2:
                scoringPosition = "";
            break;
            case ALGAE_L3:
                scoringPosition = "";
            break;
        }
    }
    
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
