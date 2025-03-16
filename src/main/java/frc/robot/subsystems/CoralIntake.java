package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.CoralIntakeConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.CoralIntakeConstants;
import frc.robot.sim.CoralIntakeSim;

public class CoralIntake extends SubsystemBase{
    //Create Motor Variables
    public TalonFX rotateMotor;
    public TalonFX spinMotor;
    CANcoder rotateCANcoder;
  
    TalonFXSimState intakePivotSim;
    TalonFXSimState intakeRollerSim;

    public VelocityVoltage spinMotorMode;
    MotionMagicVoltage rotateMotorMode;
    public DutyCycleOut rotateDutyCycleControl;

    CoralIntakeConfig coralIntakeConfig;

    //Open sensors
    DigitalInput m_BeamBreakIntakeDigital;
    Debouncer m_Debouncer;
  
    //Required for sim
    CoralIntakeSim sim;
  
    public int suckState=0;

    public CoralIntake(CoralIntakeConfig config) {
        //Constructor - only runs once

        //Instantiate
        rotateCANcoder = new CANcoder(config.kRotateCANcoderID, "canivore");
        rotateMotor = new TalonFX(config.kRotateCANID, "canivore");
        spinMotor = new TalonFX(config.kSpinCANID, "canivore");

        spinMotorMode = new VelocityVoltage(CoralIntakeConstants.kSuckSpeed);
        rotateMotorMode = new MotionMagicVoltage(CoralIntakeConstants.kIntakeStowPosition);

        m_BeamBreakIntakeDigital = new DigitalInput(config.kBeamBreakIntake);
        m_Debouncer = new Debouncer (0.05, Debouncer.DebounceType.kBoth);

        //Fill in the Instantiation
        this.configureCancoder(rotateCANcoder, config.coralIntakeCANcoderConfig);
        this.configureMechanism(spinMotor, config.coralIntakeSpinConfig);
        this.configureMechanism(rotateMotor, config.coralIntakeRotateConfig);

      
        intakePivotSim = rotateMotor.getSimState();
        intakeRollerSim = spinMotor.getSimState();

        sim = new CoralIntakeSim(config, intakePivotSim, intakeRollerSim);

        SmartDashboard.putData("Coral Intake", this);
    }

    public void configureCancoder(CANcoder coralIntakeRotate, CANcoderConfiguration config){       

    //====================================================================
    //=========================Configs====================================
    //====================================================================    

        //Start Configuring Coral Intake Motor
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

    //================================================================
    //=====================Private Methods============================
    //================================================================

    private void suck() {
    
        if (Constants.kCoralIntakeSpinMotorEnabled == true) {
            spinMotor.setControl(spinMotorMode.withVelocity(CoralIntakeConstants.kSuckSpeed));
        }
    }

    private void spit() {

        if (Constants.kCoralIntakeSpinMotorEnabled == true) {
            spinMotor.setControl(spinMotorMode.withVelocity(CoralIntakeConstants.kSpitSpeed));
        }
    }

    private void stop() {

        if (Constants.kCoralIntakeSpinMotorEnabled == true) {
            spinMotor.setControl(spinMotorMode.withVelocity(CoralIntakeConstants.kStopSpeed));
        }
    }

    private void goToPosition(double position) {

        if (Constants.kCoralIntakeRotateMotorEnabled == true) {
            rotateMotor.setControl(rotateMotorMode.withPosition(position));
        }
    }
    
    private double getPivotPosition() {

        return rotateMotor.getPosition().getValueAsDouble();
    }

    private boolean isRotateMotorAtPosition(double position) {
   
        return ((position - CoralIntakeConstants.kDeadband) <= this.getPivotPosition()) && ((position + CoralIntakeConstants.kDeadband) >= this.getPivotPosition());
    }

    private boolean isPartPresent() {


        if (m_Debouncer.calculate(m_BeamBreakIntakeDigital.get()))
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

    public void addToOrchestra(Orchestra robotOrchestra) {
        robotOrchestra.addInstrument(this.rotateMotor, 1);
        robotOrchestra.addInstrument(this.spinMotor, 1);
    }

    //=============================================================
    //====================== Commands==============================
    //=============================================================

    public Command rollerSuck() {

        return run(
            () -> {this.suck();}
        );
    }

    public Command rollerSpit() {

        return run(
            () -> {this.spit();}
        );
    }

    public Command rollerStop() {

        return runOnce(
            () -> {this.stop();}
        );
    }

    public Command intakeStow() {

        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeStowPosition);}
        ).until(isIntakeStowed);
    }

    public Command intakeRaised() {

        return run(() -> {
            this.goToPosition(CoralIntakeConstants.kIntakeRaisedPosition);
            this.spit();
        })
        .until(isIntakeRaised)
        .andThen(this.rollerStop());
        
    }

    public Command intakeFloor() {
  
        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeFloorPosition);}
        ).until(isIntakeFloored);
    }

    public Command intakeRaisedForClimb() {


        return run(
            () -> {this.goToPosition(CoralIntakeConstants.kIntakeRaisedForClimb);}
        ).until(isIntakeRaisedForClimb);
    }

    public Command clearCoralIntake() {
    return run(
        () -> {spit();}
    ).withTimeout(1)
    .andThen(this.rollerStop());

    }

    //=======================================================================
    //=========================Triggers======================================
    //=======================================================================
    public Trigger isPartPresent = new Trigger(() -> {return this.isPartPresent();});
    public Trigger isIntakeStowed= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeStowPosition);});
    public Trigger isIntakeRaised= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeRaisedPosition);});
    public Trigger isIntakeFloored= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeFloorPosition);});
    public Trigger isIntakeRaisedForClimb= new Trigger(() -> {return this.isRotateMotorAtPosition(CoralIntakeConstants.kIntakeRaisedForClimb);});
    
    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addBooleanProperty("Intake Beam Break", isPartPresent, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
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
