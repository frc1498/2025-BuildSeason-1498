package frc.robot.subsystems;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.EndEffectorConstants.movementState;
import frc.robot.constants.WristConstants;

public class EndEffector extends SubsystemBase {

    double armDesiredRotation = 0.0;
    double wristDesiredRotation = 0.0;
    double elevatorDesiredRotation = 0.0;

    DoubleSupplier armPosition;
    DoubleSupplier wristPosition;
    DoubleSupplier elevatorPosition;

    Arm arm;
    Wrist wrist;
    Elevator elevator;

    public String m_endEffectorMode = "Coral";
    String desiredPosition = "";

    movementState movement;

    public EndEffector(DoubleSupplier armPosition, DoubleSupplier wristPosition, DoubleSupplier elevatorPosition, Arm arm, Wrist wrist, Elevator elevator) {
        this.armPosition = armPosition;
        this.wristPosition = wristPosition;
        this.elevatorPosition = elevatorPosition;

        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;

        this.movement = movementState.NONE;

        SmartDashboard.putData("End Effector", this);
    }

    //========================================================
    //======================= Private ========================
    //========================================================
    private void setEndEffectorModeHere(String mode) {

        System.out.println("EndEffector:setEndEffectorModeHere:  " + mode);    
        
        this.m_endEffectorMode = mode;
    }

    private void setEndEffectorPositions(String desiredPosition) {
        this.desiredPosition = desiredPosition;

        switch (desiredPosition) {
                    case "CoralL1":
                        this.armDesiredRotation = ArmConstants.kCoralL1;
                        this.wristDesiredRotation = WristConstants.kCoralL1;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralL1;
                    break;

                    case "CoralL2":
                        this.armDesiredRotation = ArmConstants.kCoralL2;
                        this.wristDesiredRotation = WristConstants.kCoralL2;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralL2;
                    break;

                    case "CoralL3":
                        this.armDesiredRotation = ArmConstants.kCoralL3;
                        this.wristDesiredRotation = WristConstants.kCoralL3;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralL3;
                    break;

                    case "CoralL4":
                        this.armDesiredRotation = ArmConstants.kCoralL4;
                        this.wristDesiredRotation = WristConstants.kCoralL4;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralL4;
                    break;

                    case "CoralGroundPickup":
                        this.armDesiredRotation = ArmConstants.kCoralLoadFloor;
                        this.wristDesiredRotation = WristConstants.kCoralLoadFloor;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralLoadFloor;
                    break;

                    case "CoralHumanPickup":
                        this.armDesiredRotation = ArmConstants.kCoralLoadHuman;
                        this.wristDesiredRotation = WristConstants.kCoralLoadHuman;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralLoadHuman;
                    break;

                    case "CoralStow":
                        this.armDesiredRotation = ArmConstants.kCoralStow;
                        this.wristDesiredRotation = WristConstants.kCoralStow;
                        this.elevatorDesiredRotation = ElevatorConstants.kCoralStow;
                    break;

                    default:
                        this.armDesiredRotation = armPosition.getAsDouble();
                    break;
        }
    }

    private String getDesiredPosition() {
        return this.desiredPosition;
    }

    public double getArmRotation() {
        return this.armDesiredRotation;
    }

    public DoubleSupplier armRotation() {
        return this::getArmRotation;
    }

    public double getWristRotation() {
        return this.wristDesiredRotation;
    }

    public DoubleSupplier wristRotation() {
        return this::getWristRotation;
    }

    public double getElevatorRotation() {
        return this.elevatorDesiredRotation;
    }

    public DoubleSupplier elevatorRotation() {
        return this::getElevatorRotation;
    }

    public void setMovementState() {
        if ((armPosition.getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) // We are in front of the robot, rotating past rearsafe
        {
            this.movement = movementState.FRONT_PAST_REAR_SAFE;


        } else if ((armPosition.getAsDouble() > ArmConstants.kRearSafe) && (armDesiredRotation < ArmConstants.kFrontSafe)) //We are behind the robot, rotating past frontsafe
        {
            this.movement = movementState.REAR_PAST_FRONT_SAFE;


        } else if ((armPosition.getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the front of the robot, rotating middle
        {
            this.movement = movementState.FRONT_MIDDLE;


        } else if ((armPosition.getAsDouble() > ArmConstants.kIntakeSafe) && (armDesiredRotation < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the rear of the robot, rotating middle
        {
            this.movement = movementState.REAR_MIDDLE;


        } else if ((armPosition.getAsDouble() > ArmConstants.kFrontSafe) && (armPosition.getAsDouble() < ArmConstants.kRearSafe) && (armDesiredRotation < ArmConstants.kFrontSafe))  //We are in the middle of the robot, rotating forward
        {
            this.movement = movementState.MIDDLE_PAST_FRONT_SAFE;


        } else if ((armPosition.getAsDouble() > ArmConstants.kFrontSafe) && (armPosition.getAsDouble() < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kIntakeSafe))  //We are in the middle of the robot
        {
            this.movement = movementState.MIDDLE_PAST_REAR_SAFE;  
        
        } else if ((armPosition.getAsDouble() > ArmConstants.kFrontSafe) && (armPosition.getAsDouble() < ArmConstants.kIntakeSafe) && (armDesiredRotation < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the middle of the robot
        {
            this.movement = movementState.MIDDLE_MIDDLE;  //For the rare situation where we are in the middle and tell it to go to the middle
        
        
        } else 
        {
            //We are in an unknown position.  Stop.
            this.movement = movementState.NONE;
        }
    }

    public movementState getMovementState() {
        return this.movement;
    }

    public String getMovementStateString() {
        return this.movement.toString();
    }

    public Boolean GetEndEffectorMode() {

        return (m_endEffectorMode == "Algae");
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {

        return runOnce(() -> {this.setEndEffectorModeHere(mode);});
    }

    public Command whatIsEndEffectorMode() {

        return runOnce(() -> {this.GetEndEffectorMode();});
    }

    public Command setPositions(String desiredLocation) {
        return runOnce(() -> {this.setEndEffectorPositions(desiredLocation);});
    }

    public Command setMovement() {
        return runOnce(() -> {this.setMovementState();});
    }

    public Command prepForMovement(String desiredLocation) {
        return this.setPositions(desiredLocation).andThen(this.setMovement());
    }

    public Command updateDesired() {
        return this.toDesired((() -> {return this.getArmRotation();}), (() -> {return this.getWristRotation();}), (() -> {return this.getElevatorRotation();}));
    }

    public Command toDesired(DoubleSupplier armPosition, DoubleSupplier wristPosition, DoubleSupplier elevatorPosition) {

        return arm.toArmPosition(armPosition.getAsDouble())
            .alongWith(wrist.toWristPosition(wristPosition.getAsDouble()))
            .alongWith(elevator.toElevatorPosition(elevatorPosition.getAsDouble()));
    }

    public Command test() {
        return new Command() {
            @Override
            public void initialize() {

            }
        };
    }

    /*
    public Supplier<String> endEffectorMode() {
        return this::GetEndEffectorMode; 
    }
    */

    //======================================================
    //===================Triggers===========================
    //======================================================
    public final Trigger isModeAlgae = new Trigger(() -> {return this.GetEndEffectorMode();});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Passed Arm Position", armPosition, null);
        builder.addStringProperty("Desired Location", this::getDesiredPosition, null);
        builder.addStringProperty("Movement State", this::getMovementStateString, null);
        builder.addDoubleProperty("Desired Arm Position", () -> {return this.armDesiredRotation;}, null);
        builder.addDoubleProperty("Desired Wrist Position", () -> {return this.wristDesiredRotation;}, null);
        builder.addDoubleProperty("Desired Elevator Position", () -> {return this.elevatorDesiredRotation;}, null);
    }
}
