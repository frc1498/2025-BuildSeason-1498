package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ElevatorConfig;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.sim.ElleySim;

public class Elevator extends SubsystemBase {
    //Declare Variables
    public TalonFX elevatorDriveFront;
  
    TalonFXSimState elevatorDriveFrontSim;
  
    MotionMagicVoltage posControl;
    
    //PositionVoltage posControl;
    public DutyCycleOut rotateDutyCycleControl;

    private double desiredPosition;

    ElevatorConfig config;
    ElleySim sim;

    public Elevator(ElevatorConfig config) {
        //Constructor.

        //Potentially unneeded step.
        //this.config = config;

        //Instantiate
        elevatorDriveFront = new TalonFX(config.kElevatorDriveFrontCANID, "canivore");

        //Fill In the Instantiation
        this.configureMechanism(elevatorDriveFront, config.frontConfig);

        elevatorDriveFrontSim = elevatorDriveFront.getSimState();

        posControl = new MotionMagicVoltage(0);   

        sim = new ElleySim(config, elevatorDriveFrontSim);
        
        SmartDashboard.putData("Elevator", this);
    }

   public void configureMechanism(TalonFX mechanism, TalonFXConfiguration motorConfig){     

    //=====================================================
    //================Configuration========================
    //=====================================================  

        //Start Configuring Climber Motor
        StatusCode mechanismStatus = StatusCode.StatusCodeNotInitialized;

        for(int i = 0; i < 5; ++i) {
            mechanismStatus = mechanism.getConfigurator().apply(motorConfig);
            if (mechanismStatus.isOK()) break;
        }
        if (!mechanismStatus.isOK()) {
            System.out.println("Could not configure device. Error: " + mechanismStatus.toString());
        }
    }
    //=======================================================
    //==========================Private======================
    //=======================================================
    private void elevatorDriveToPosition(double position) {
        this.desiredPosition = position;

        if (Constants.kElevatorExtendMotorEnabled == true) {
            elevatorDriveFront.setControl(posControl.withPosition(position));
        }
    }

    private double getElevatorPosition() {

        return elevatorDriveFront.getPosition().getValueAsDouble();
    }

    private double getDesiredPosition() {

        return this.desiredPosition;
    }

    private boolean isElevatorAtPosition(double position) {

        return (position - ElevatorConstants.kDeadband) <= this.getElevatorPosition() && (position + ElevatorConstants.kDeadband) >= this.getElevatorPosition();
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }
    //======================================================================
    //=====================Public Commands==================================
    //======================================================================
    
    public Command elevatorFrontSafe() {
               
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kFrontSafe);}
        ).until(this.isElevatorFrontSafe);
    }
    
    public Command elevatorRearSafe() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kRearSafe);}
        ).until(this.isElevatorRearSafe);
    }

    public Command elevatorCoralStow() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralStow);}
        ).until(this.isElevatorCoralStow);
    }

    public Command elevatorCoralLoadFloor() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralLoadFloor);}
        ).until(this.isElevatorCoralLoadFloor);
    }

    public Command elevatorCoralLoadHuman() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralLoadHuman);}
        ).until(this.isElevatorCoralLoadHuman);
    }

    public Command elevatorCoralL1() {      

        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL1);}
        ).until(this.isElevatorCoralL1);
    }

    public Command elevatorCoralL2() {

        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL2);}
        ).until(this.isElevatorCoralL2);
    }
    
    public Command elevatorCoralL3() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL3);}
        ).until(this.isElevatorCoralL3);
    }

    public Command elevatorCoralL4() {

        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kCoralL4);}
        ).until(this.isElevatorCoralL4);
    }

    public Command elevatorAlgaeStow() {

        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeStow);}
        ).until(this.isElevatorAlgaeStow);
    }

    public Command elevatorAlgaeLoadFloor() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeLoadFloor);}
        ).until(this.isElevatorAlgaeLoadFloor);
    }

    public Command elevatorAlgaeL2() {
 
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeL2);}
        ).until(this.isElevatorAlgaeL2);
    }
    
    public Command elevatorAlgaeL3() {
 
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeL3);}
        ).until(this.isElevatorAlgaeL3);
    }
    
    public Command elevatorAlgaeBarge() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeBarge);}
        ).until(this.isElevatorAlgaeBarge);
    }

    public Command elevatorAlgaeProcessor() {
        
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kAlgaeProcessor);}
        ).until(this.isElevatorAlgaeProcessor);
    }

    public Command elevatorPosition(Supplier<Double> position) {
        
        return run(
            () -> {this.elevatorDriveToPosition(position.get());}
        );
    }

    public Command toElevatorPosition(double position) {
        
        return run(
            () -> {this.elevatorDriveToPosition(position);}
        );
    }

    public DoubleSupplier getElevatorRotation() {

        return this::getElevatorPosition;
    }

    //======================================================
    //==============Triggers for Elevator Coral=============
    //======================================================
    public final Trigger isElevatorCoralStow = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralStow);});
    public final Trigger isElevatorCoralLoadFloor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralLoadFloor);});
    public final Trigger isElevatorCoralLoadHuman = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralLoadHuman);});
    public final Trigger isElevatorCoralL1 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL1);});
    public final Trigger isElevatorCoralL2 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL2);});
    public final Trigger isElevatorCoralL3 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL3);});
    public final Trigger isElevatorCoralL4 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kCoralL4);});

    //=====================================================
    //==============Triggers for Elevator Algae============
    //=====================================================
    public final Trigger isElevatorAlgaeStow = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeStow);});
    public final Trigger isElevatorAlgaeLoadFloor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeLoadFloor);});
    public final Trigger isElevatorAlgaeL2 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeL2);});
    public final Trigger isElevatorAlgaeL3 = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeL3);});
    public final Trigger isElevatorAlgaeBarge = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeBarge);});
    public final Trigger isElevatorAlgaeProcessor = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kAlgaeProcessor);});

    //=====================================================
    //===============Triggers General======================
    //=====================================================
    public final Trigger isElevatorFrontSafe = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kFrontSafe);});
    public final Trigger isElevatorRearSafe = new Trigger(() -> {return this.isElevatorAtPosition(ElevatorConstants.kRearSafe);});
 
    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Desired Position", this::getDesiredPosition, null);
        builder.addDoubleProperty("Current Position", this::getElevatorPosition, null);
        builder.addBooleanProperty("Is Elevator at Coral L1", isElevatorCoralL1, null);
        builder.addBooleanProperty("Is Elevator at Coral L2", isElevatorCoralL2, null);    
        builder.addBooleanProperty("Is Elevator at Coral L3", isElevatorCoralL3, null);
        builder.addBooleanProperty("Is Elevator at Coral Load Floor", isElevatorCoralLoadFloor, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);  
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void simulationInit() {
        sim = new ElleySim(config, elevatorDriveFrontSim);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        sim.simulationPeriodic();
    }
}
