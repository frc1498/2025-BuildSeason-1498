package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ClimberConfig;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase{
    //Declare Variables
    public TalonFX climberRotate;
    MotionMagicVoltage rotateControl;
    public DutyCycleOut rotateDutyCycleControl;
    Servo climberServo;
    double desiredServoPosition;

    //TalonFX climberSpin;
    //VelocityVoltage spinControl;

    //This was causing an error - had to comment out
    //TalonFXSimState climberDriveFrontSim = climberRotate.getSimState();
    //ClimberSim sim;
   
    public boolean climberEnabled = false;

    public Climber(ClimberConfig config) {
        //Constructor

        //Instantiate
        climberRotate = new TalonFX(config.kclimberRotateCANID, "canivore");
        rotateControl = new MotionMagicVoltage(ClimberConstants.kClimberStowed);

        climberServo = new Servo(9);

        desiredServoPosition = 0;

        //Fill in Instantiation
        this.configureMechanism(climberRotate, config.climberRotateConfig);
        //this.configureMechanism(climberSpin, config.climberSpinConfig);

        SmartDashboard.putData("Climber", this);
    }
   
    public void configureMechanism(TalonFX mechanism, TalonFXConfiguration config){     

    //===========================================================
    //====================Configuration==========================
    //===========================================================  

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

    //===========================================================
    //=======================Private=============================
    //===========================================================

    private void climberLatch() {
        this.desiredServoPosition = ClimberConstants.kServoLatch;
        climberServo.set(ClimberConstants.kServoLatch);
    }

    private void climberUnLatch() {
        this.desiredServoPosition = ClimberConstants.kServoUnLatch;
        climberServo.set(ClimberConstants.kServoUnLatch);
    }


    private void climberDriveToPosition(double position) {
        if (Constants.kClimberRotateMotorEnabled == true) {
            climberRotate.setControl(rotateControl.withPosition(position));
        }
    }

    private boolean isClimberAtPosition(double position) {
        /* climber is removed (position-ClimberConstants.kDeadband) <= getClimberPosition()) &&
         ((position+ClimberConstants.kDeadband) >= getClimberPosition() */
        
        return (false);
    }

    private double getClimberPosition(){
            return climberRotate.getPosition().getValueAsDouble();       
    }

    private double getServoPosition() {
        return this.climberServo.get();
    }

    private double getDesiredServoPosition() {
        return this.desiredServoPosition;
        
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    private void climberEnable(){
        climberEnabled=true;
    }

    public void addToOrchestra(Orchestra robotOrchestra) {
        robotOrchestra.addInstrument(this.climberRotate, 1);
    }

    //===============================================================
    //=====================Commands==================================
    //===============================================================
    public Command commandClimberLatch() {
        return runOnce(
            () -> {this.climberLatch();}).withName("Climber Latch");
    }

    public Command commandClimberUnLatch() {
        return runOnce(
            () -> {this.climberUnLatch();}).withName("Climber Unlatch");
    }

    public Command toClimberStow() {
        return run(
            () -> {this.climberDriveToPosition(ClimberConstants.kClimberStowed);}).until(isClimberStowed).withName("To Climber Stow");
    }

    public Command toClimberReady() {
        return run(            
            () -> {this.climberDriveToPosition(ClimberConstants.kClimberReady);}).until(isClimberReady).withName("To Climber Ready");
    }

    public Command toClimberComplete() {

        //    System.out.println("=============Command toClimberComplete===============");

        
        return run(
            () -> {this.climberDriveToPosition(ClimberConstants.kClimberComplete);}).until(isClimberComplete).withName("To Climber Complete");
    }

    public Command climberTriggered() {
        return runOnce(
            () -> {this.climberEnable();}).withName("Climber Triggered");  
    
    }



    //===============================================================
    //======================Triggers=================================
    //===============================================================
    public final Trigger isClimberComplete = new Trigger(() -> {return this.isClimberAtPosition(ClimberConstants.kClimberComplete);});
    public final Trigger isClimberStowed = new Trigger(() -> {return this.isClimberAtPosition(ClimberConstants.kClimberStowed);});
    public final Trigger isClimberReady = new Trigger(() -> {return this.isClimberAtPosition(ClimberConstants.kClimberReady);});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addDoubleProperty("Servo Position", this::getServoPosition, null);
        builder.addDoubleProperty("Desired Servo Position", this::getDesiredServoPosition, null);
        builder.addBooleanProperty("Climber Ready?", isClimberReady, null);
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
