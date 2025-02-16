package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ArmConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.EndEffector;

public class EndEffectorCommand {
    ElevatorConfig elevatorConfig;
    public Elevator elevator;
    ArmConfig armConfig;
    public Arm arm;
    WristConfig wristConfig;
    public Wrist wrist;
    EndEffector endEffector;

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

    public String endEffectorMode = "None";
    public String previousDesiredLocation = "";
    public double armDesiredRotation = 0.0;
    public double wristDesiredRotation = 0.0;
    public double elevatorDesiredRotation = 0.0;
    
    public double armDesiredRotation2 = 0.0;
    public double wristDesiredRotation2 = 0.0;
    public double elevatorDesiredRotation2 = 0.0;

    public EndEffectorCommand() {
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

    private Boolean isEndEffectorAtPosition(String desiredLocation) {
    //need previous desired location

        if (desiredLocation != previousDesiredLocation){
            if (endEffector.isModeAlgae.getAsBoolean()){     //Are we Algae mode     
                switch (desiredLocation) {
                    case "L1orProcessor":
                        armDesiredRotation2 = ArmConstants.kAlgaeProcessor;
                        wristDesiredRotation2 = WristConstants.kAlgaeProcessor;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeProcessor;
                    break;
                    case "L2":
                        armDesiredRotation2 = ArmConstants.kAlgaeL2;
                        wristDesiredRotation2 = WristConstants.kAlgaeL2;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeL2;
                    break;
                    case "L3":
                        armDesiredRotation2 = ArmConstants.kAlgaeL3;
                        wristDesiredRotation2 = WristConstants.kAlgaeL3;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeL3;
                    break;
                    case "L4orBarge":
                        armDesiredRotation2 = ArmConstants.kAlgaeBarge;
                        wristDesiredRotation2 = WristConstants.kAlgaeBarge;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeBarge;
                    break;
                }
            } else if (!endEffector.isModeAlgae.getAsBoolean()){  //Are we coral mode
                switch (desiredLocation) {
                    case "L1orProcessor":
                        armDesiredRotation2 = ArmConstants.kCoralL1;
                        wristDesiredRotation2 = WristConstants.kCoralL1;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL1;
                    break;
                    case "L2":
                        armDesiredRotation2 = ArmConstants.kCoralL2;
                        wristDesiredRotation2 = WristConstants.kCoralL2;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL2;
                    break;
                    case "L3":
                        armDesiredRotation2 = ArmConstants.kCoralL3;
                        wristDesiredRotation2 = WristConstants.kCoralL3;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL3;
                    break;
                    case "L4orBarge":
                        armDesiredRotation2 = ArmConstants.kCoralL4;
                        wristDesiredRotation2 = WristConstants.kCoralL4;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL4;
                    break;
                }
            }
        }

        if (EndEffectorConstants.kEndEffectorPrint){
            System.out.println("=========================End Effector In Position?=====================");
            System.out.println("Algae Mode:" + endEffector.isModeAlgae.getAsBoolean());
            System.out.println("End Effector Desired Location: " + desiredLocation);
            System.out.println("wrist Desired Rotation:" + wristDesiredRotation2);
            System.out.println("arm Desired Rotation:" + armDesiredRotation2);
            System.out.println("arm Desired Rotation:" + elevatorDesiredRotation2);
            System.out.println("wrist Actual Rotation:" + wrist.getWristRotation().getAsDouble());
            System.out.println("arm Actual Rotation:" + arm.getArmRotation().getAsDouble());
            System.out.println("elevator Actual Rotation:" + elevator.getElevatorRotation().getAsDouble());
        }

        return  (((wristDesiredRotation2 - WristConstants.kDeadband) <= wrist.getWristRotation().getAsDouble())
                && ((wristDesiredRotation2 + WristConstants.kDeadband) >= wrist.getWristRotation().getAsDouble())
                && ((armDesiredRotation2 - ArmConstants.kDeadband) <= arm.getArmRotation().getAsDouble()) 
                && ((armDesiredRotation2 + ArmConstants.kDeadband) >= arm.getArmRotation().getAsDouble())
                && ((elevatorDesiredRotation2 - ElevatorConstants.kDeadband) <= elevator.getElevatorRotation().getAsDouble()) 
                && ((elevatorDesiredRotation2 + ElevatorConstants.kDeadband) >= elevator.getElevatorRotation().getAsDouble()));
    
    }


    //==========================================================
    //=====================Commands=============================
    //==========================================================
    public Command moveEndEffector(String desiredLocation) {

        if (desiredLocation != previousDesiredLocation){
            if (endEffector.isModeAlgae.getAsBoolean()){  //Check to see if we are algae mode      
                switch (desiredLocation) {
                    case "L1orProcessor":
                        armDesiredRotation = ArmConstants.kAlgaeProcessor;
                        wristDesiredRotation = WristConstants.kAlgaeProcessor;
                        elevatorDesiredRotation = ElevatorConstants.kAlgaeProcessor;
                    break;
                    case "L2":
                        armDesiredRotation = ArmConstants.kAlgaeL2;
                        wristDesiredRotation = WristConstants.kAlgaeL2;
                        elevatorDesiredRotation = ElevatorConstants.kAlgaeL2;
                    break;
                    case "L3":
                        armDesiredRotation = ArmConstants.kAlgaeL3;
                        wristDesiredRotation = WristConstants.kAlgaeL3;
                        elevatorDesiredRotation = ElevatorConstants.kAlgaeL3;
                    break;
                    case "L4orBarge":
                        armDesiredRotation = ArmConstants.kAlgaeBarge;
                        wristDesiredRotation = WristConstants.kAlgaeBarge;
                        elevatorDesiredRotation = ElevatorConstants.kAlgaeBarge;
                    break;
                    default:
                    break;
                }
            } else if (!endEffector.isModeAlgae.getAsBoolean()){ //Check to see if we are coral mode
                switch (desiredLocation) {
                    case "L1orProcessor":
                        armDesiredRotation = ArmConstants.kCoralL1;
                        wristDesiredRotation = WristConstants.kCoralL1;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL1;
                    break;
                    case "L2":
                        armDesiredRotation = ArmConstants.kCoralL2;
                        wristDesiredRotation = WristConstants.kCoralL2;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL2;
                    break;
                    case "L3":
                        armDesiredRotation = ArmConstants.kCoralL3;
                        wristDesiredRotation = WristConstants.kCoralL3;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL3;
                    break;
                    case "L4orBarge":
                        armDesiredRotation = ArmConstants.kCoralL4;
                        wristDesiredRotation = WristConstants.kCoralL4;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL4;
                    break;
                    default:
                    break;
                }
            }
        }

        //Counter clockwise is positive on all rotations, so algae pickup or front / low is the lowest we go
        if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) // We are in front of the robot, rotating past rearsafe
        {
            movement = movementState.FRONT_PAST_REAR_SAFE;
        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) //We are behind the robot, rotating past frontsafe
        {
            movement = movementState.REAR_PAST_FRONT_SAFE;
        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the front of the robot, rotating middle
        {
            movement = movementState.FRONT_MIDDLE;
        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the rear of the robot, rotating middle
        {
            movement = movementState.REAR_MIDDLE;
        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the middle of the robot, rotating forward
        {
            movement = movementState.MIDDLE_PAST_FRONT_SAFE;
        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe))  //We are in the middle of the robot, rotating rearward
        {
            movement = movementState.MIDDLE_PAST_REAR_SAFE;
        } else 
        {
            //We are in an unknown position.  Stop.
            movement = movementState.NONE;
        }

        if (EndEffectorConstants.kEndEffectorPrint){
            System.out.println("=========================End Effector Moving=====================");
            System.out.println("Algae Mode:" + endEffector.isModeAlgae.getAsBoolean());
            System.out.println("End Effector Desired Location: " + desiredLocation);
            System.out.println("wrist Desired Rotation:" + wristDesiredRotation);
            System.out.println("arm Desired Rotation:" + armDesiredRotation);
            System.out.println("arm Desired Rotation:" + elevatorDesiredRotation);
            System.out.println("wrist Actual Rotation:" + wrist.getWristRotation().getAsDouble());
            System.out.println("arm Actual Rotation:" + arm.getArmRotation().getAsDouble());
            System.out.println("elevator Actual Rotation:" + elevator.getElevatorRotation().getAsDouble());
        }

        previousDesiredLocation = desiredLocation;

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
                return this.toDesired(wrist.getWristRotation().getAsDouble(), arm.getArmRotation().getAsDouble(), elevator.getElevatorRotation().getAsDouble());     
            default:
                return this.toDesired(wrist.getWristRotation().getAsDouble(), arm.getArmRotation().getAsDouble(), elevator.getElevatorRotation().getAsDouble());     
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

//====================Goto Coral Locations==============================
    public Command toCoralL1() {
        return this.moveEndEffector("CoralL1");    
    }
    
    public Command toCoralL2() {
        return  this.moveEndEffector("CoralL2");   
    }

    public Command toCoralL3() {
        return this.moveEndEffector("CoralL3");
    }

    public Command toCoralL4() {
        return this.moveEndEffector("CoralL4");
    }

    public Command toCoralGroundPickup() {
        return this.moveEndEffector("CoralGroundPickup");
    }

    public Command toCoralHumanPickup() {
        return this.moveEndEffector("CoralHumanPickup");
    }

    public Command toCoralStow() {
        return this.moveEndEffector("CoralStow");
    }

    //==================Goto Alage Locations================================
    public Command toAlgaePickup() {
        return this.moveEndEffector("AlgaePickup");
    }

    public Command toAlgaeScoreBarge() {
        return this.moveEndEffector("AlgaeScoreBarge");
    }
    
    public Command toAlgaeL2() {
        return this.moveEndEffector("AlgaeL2");
    }
    
    public Command toAlgaeL3() {
        return this.moveEndEffector("AlgaeL3");
    }
    
    public Command toAlgaeProcessor() {
        return this.moveEndEffector("AlgaeProcessor");
    }

    //======================================================
    //========================Triggers======================
    //======================================================
    public Trigger isEndEffectorAtCoralL1 = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralL1");});
    public Trigger isEndEffectorAtCoralL2 = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralL2");});
    public Trigger isEndEffectorAtCoralL3 = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralL3");});
    public Trigger isEndEffectorAtCoralL4 = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralL4");});
    public Trigger isEndEffectorAtCoralGroundPickup = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralGroundPickup");});
    public Trigger isEndEffectorAtCoralHumanPickup = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralHumanPickup");});
    public Trigger isEndEffectorAtCoralStow = new Trigger(() ->{return this.isEndEffectorAtPosition("CoralStow");});

    public Trigger isEndEffectorAlgaeFloor = new Trigger(() ->{return this.isEndEffectorAtPosition("AlgaeFloor");});
    public Trigger isEndEffectorScoreBarge = new Trigger(() ->{return this.isEndEffectorAtPosition("AlgaeBarge");});
    public Trigger isEndEffectorAlgaeL2 = new Trigger(() ->{return this.isEndEffectorAtPosition("AlgaeL2");});
    public Trigger isEndEffectorAlgaeL3 = new Trigger(() ->{return this.isEndEffectorAtPosition("AlgaeL3");});
    public Trigger isEndEffectorAlgaeProcessor = new Trigger(() ->{return this.isEndEffectorAtPosition("AlgaeProcessor");});

}
