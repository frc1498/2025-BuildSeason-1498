package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.constants.ArmConstants;

public class EndEffector {
    ElevatorConfig elevatorConfig;
    Elevator elevator;
    ArmConfig armConfig;
    Arm arm;
    WristConfig wristConfig;
    Wrist wrist;

    enum movementState {
        FRONT_PAST_REAR_SAFE,
        REAR_PAST_FRONT_SAFE,
        FRONT_MIDDLE,
        REAR_MIDDLE,
        MIDDLE_PAST_FRONT_SAFE,
        MIDDLE_PAST_REAR_SAFE,
        NONE
    }

    movementState movement;

    public String endEffectorMode="None";

    public EndEffector() {
        elevatorConfig = new ElevatorConfig();
        elevator = new Elevator(elevatorConfig);
        armConfig = new ArmConfig();
        arm = new Arm(armConfig);
        wristConfig = new WristConfig();
        wrist = new Wrist(wristConfig);

        movement = movementState.NONE;
    }

    //==========================================================
    //=====================Configuration========================
    //==========================================================



    //==========================================================
    //======================Private=============================
    //==========================================================

    private void setEndEffectorMode(String mode) {
        this.endEffectorMode = mode;
    }

    private boolean isEndEffectorAlgae() {
        return (endEffectorMode == "Algae");
    }

    //==========================================================
    //=====================Commands=============================
    //==========================================================

    public Command callEndEffector(){
        if (isEndEffectorModeAlgae) {

        }

    }

    public Command moveEndEffector(double wristDesiredRotation, double armDesiredRotation, double elevatorDesiredRotation) {

        //Counter clockwise is positive on all rotations, so algae pickup or front / low is the lowest we go
        if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) // We are in front of the robot, rotating past rearsafe
        {
            movement = movementState.FRONT_PAST_REAR_SAFE;
        } else if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) //We are behind the robot, rotating past frontsafe
        {
            movement = movementState.REAR_PAST_FRONT_SAFE;
        } else if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the front of the robot, rotating middle
        {
            movement = movementState.FRONT_MIDDLE;
        } else if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the rear of the robot, rotating middle
        {
            movement = movementState.REAR_MIDDLE;
        } else if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the middle of the robot, rotating forward
        {
            movement = movementState.MIDDLE_PAST_FRONT_SAFE;
        } else if ((arm.armRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the middle of the robot, rotating rearward
        {
            movement = movementState.MIDDLE_PAST_REAR_SAFE;
        } else 
        {
            //We are in an unknown position.  Stop.
            movement = movementState.NONE;
        }

        switch (movement) {
            case FRONT_PAST_REAR_SAFE:   
                return this.toFrontSafe().andThen(this.toRearSafe()).andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case REAR_PAST_FRONT_SAFE:
                return this.toRearSafe().andThen(this.toFrontSafe()).andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case FRONT_MIDDLE:
                return this.toFrontSafe().andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case REAR_MIDDLE:
                return this.toRearSafe().andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case MIDDLE_PAST_FRONT_SAFE:
                return this.toFrontSafe().andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case MIDDLE_PAST_REAR_SAFE:
                return this.toRearSafe().andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation));
            case NONE:
                return this.toDesired(wristLastRotation, armLastRotation, elevatorLastRotation);

        }
    }

    public Command toFrontSafe() {
        return 
            elevator.elevatorFrontSafe().until(elevator.isElevatorFrontSafe)
            .andThen(wrist.wristFrontSafe()).until(wrist.isWristFrontSafe)
            .andThen(arm.armFrontSafe()).until(arm.isArmFrontSafe);
        
    }


    public Command toRearSafe() {
        return 
            elevator.elevatorRearSafe().until(elevator.isElevatorRearSafe)
            .andThen(wrist.wristRearSafe()).until(wrist.isWristRearSafe)
            .andThen(arm.armRearSafe()).until(arm.isArmRearSafe);

    }

    public Command toDesired(double wristDesiredRotation, double armDesiredRotation, double elevatorDesiredRotation) {
        return 
            Commands.parallel(elevator.toElevatorPosition(elevatorDesiredRotation),
            wrist.toWristPosition(wristDesiredRotation),
            arm.toArmPosition(armDesiredRotation));
        
    }

    public Command endEffectorMode(String mode) {
        return new InstantCommand(() -> {this.setEndEffectorMode(mode);});
    }

    //======================================================
    //========================Triggers======================
    //======================================================


}
