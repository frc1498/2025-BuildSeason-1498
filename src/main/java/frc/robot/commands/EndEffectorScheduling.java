package frc.robot.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.EndEffectorConstants.movementState;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;

public class EndEffectorScheduling {

    String desiredLocation;
    String previousDesiredLocation;
    double armDesiredRotation;
    double wristDesiredRotation;
    double elevatorDesiredRotation;
    movementState movement = movementState.MIDDLE_MIDDLE;
    boolean stateFinished = false;

    Arm arm;
    Wrist wrist;
    Elevator elevator;
    EndEffector state;

    public EndEffectorScheduling(Arm armSubsystem, Wrist wristSubsystem, Elevator elevatorSubsystem, EndEffector state) {
        this.arm = armSubsystem;
        this.wrist = wristSubsystem;
        this.elevator = elevatorSubsystem;
        this.state = state;
    }

    private String getMovementString() {
        return this.movement.toString();
    }

    public Command scheduledMovement(Supplier<movementState> movementState, DoubleSupplier armMovement, DoubleSupplier wristMovement, DoubleSupplier elevatorMovement) {
        return null;
        //BAD! FIX!
    }

    public Command movementSelector(Supplier<movementState> movementState) {
        return new SelectCommand<>(
            Map.ofEntries(
                Map.entry(movement.NONE, this.toFrontSafe().andThen(this.toRearSafe())),
                Map.entry(movement.FRONT_MIDDLE, this.toFrontSafe())   
            )
            , movementState
        );
    }

    /*@Override
    public void initialize() {
        this.movement = state.getMovementState();
        this.armDesiredRotation = state.getArmRotation();
        this.wristDesiredRotation = state.getWristRotation();
        this.elevatorDesiredRotation = state.getElevatorRotation();
    }

    @Override
    public void execute() {
        switch (movement) {
            case FRONT_PAST_REAR_SAFE:  
                Commands.parallel((this.toFrontSafe()
                    .andThen(this.toRearSafe())
                    .andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            break;
            case REAR_PAST_FRONT_SAFE:
                Commands.parallel((this.toRearSafe()
                    .andThen(this.toFrontSafe())
                    .andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            break;
            case FRONT_MIDDLE:
                Commands.parallel((this.toFrontSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            break;
            case REAR_MIDDLE:
                Commands.parallel((this.toRearSafe().
                    andThen(this.toIntakeSafe()).andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation),
                    Commands.print("(arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe): " + (arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe)),
                    Commands.print("(armDesiredRotation < ArmConstants.kIntakeSafe): " + (armDesiredRotation < ArmConstants.kIntakeSafe) ),
                    Commands.print("(armDesiredRotation > ArmConstants.kFrontSafe): " + (armDesiredRotation > ArmConstants.kFrontSafe)));
            break;          
            case MIDDLE_PAST_FRONT_SAFE:
                Commands.parallel((this.toFrontSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement));
            break;
            case MIDDLE_PAST_REAR_SAFE:          
                Commands.parallel((this.toIntakeSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation),
                    Commands.print("(arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe): " + (arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe)),
                    Commands.print("(armDesiredRotation < ArmConstants.kIntakeSafe): " + (armDesiredRotation < ArmConstants.kIntakeSafe) ),
                    Commands.print("(armDesiredRotation > ArmConstants.kFrontSafe): " + (armDesiredRotation > ArmConstants.kFrontSafe)));
            break;
            case MIDDLE_MIDDLE:          
                Commands.parallel((this.toMiddleSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation));
            break;
            case NONE:
                //Do Nothing?
                this.toFrontSafe().andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
                stateFinished = true;
            break;
            default:                     
                //This sets everything to where it currently is because something is wrong.....
                Commands.parallel((this.toDesired(wrist.getWristRotation().getAsDouble(), 
                    arm.getArmRotation().getAsDouble(), 
                    elevator.getElevatorRotation().getAsDouble())),
                    Commands.print("EndEffectorCommand:moveEndEffector: DEFAULT!!!!: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation),
                    Commands.print("(arm.getArmRotation().getAsDouble() > ArmConstants.kFrontSafe): " + (arm.getArmRotation().getAsDouble() > ArmConstants.kFrontSafe)),
                    Commands.print("(arm.getArmRotation().getAsDouble() < ArmConstants.kIntakeSafe): " + (arm.getArmRotation().getAsDouble() < ArmConstants.kIntakeSafe) ),
                    Commands.print("armDesiredRotation > ArmConstants.kIntakeSafe: " + (armDesiredRotation > ArmConstants.kIntakeSafe)));
            break;
         }
    }

    @Override
    public boolean isFinished() {
        return stateFinished;
    }*/

    public Command toMiddleSafe() {

        return Commands.parallel(
            elevator.elevatorMiddleSafe(),
            Commands.print("EndEffectorCommand:toMiddleSafe"));    

    }

    public Command toFrontSafe() {

        return Commands.parallel((
            elevator.elevatorFrontSafe()
            .andThen(wrist.wristFrontSafe())
            .andThen(arm.armFrontSafe())),
            Commands.print("EndEffectorCommand:toFrontSafe"));       
    }

    public Command toIntakeSafe() {
        
        return Commands.parallel((
            elevator.elevatorRearSafe()
            .andThen(wrist.wristRearSafe())
            .andThen(arm.armRearSafe())),
            Commands.print("EndEffectorCommand:toIntakeSafe"));
    }


    public Command toRearSafe() {
        
        return Commands.parallel((
            elevator.elevatorRearSafe()
            .andThen(wrist.wristRearSafe())
            .andThen(arm.armRearSafe())),
            Commands.print("EndEffectorCommand:toRearSafe"));
    }

    public Command toDesired(double wristDesiredRotation, double armDesiredRotation, double elevatorDesiredRotation) {

        return Commands.parallel((
            Commands.parallel(elevator.toElevatorPosition(elevatorDesiredRotation),
            wrist.toWristPosition(wristDesiredRotation),
            arm.toArmPosition(armDesiredRotation))),
            Commands.print("EndEffectorCommand:toDesired"));        
    }
    
    /*
    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addStringProperty("State in Command", this::getMovementString, null);
        builder.addDoubleProperty("Desired Arm", () -> {return this.armDesiredRotation;}, null);
        builder.addDoubleProperty("Desired Wrist", () -> {return this.wristDesiredRotation;}, null);
        builder.addDoubleProperty("Desired Elevator", () -> {return this.elevatorDesiredRotation;}, null);
    }*/
}
