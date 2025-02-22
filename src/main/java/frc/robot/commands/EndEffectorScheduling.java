package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class EndEffectorScheduling extends Command{

    String desiredLocation;
    String previousDesiredLocation;
    double armDesiredRotation;
    double wristDesiredRotation;
    double elevatorDesiredRotation;

    Arm arm;
    Wrist wrist;
    Elevator elevator;

    enum movementState {
        FRONT_PAST_REAR_SAFE,
        REAR_PAST_FRONT_SAFE,
        FRONT_MIDDLE,
        REAR_MIDDLE,
        MIDDLE_PAST_FRONT_SAFE,
        MIDDLE_PAST_REAR_SAFE,
        MIDDLE_MIDDLE,
        NONE
    }

    movementState movement;

    public EndEffectorScheduling(String desiredLocation, Arm armSubsystem, Wrist wristSubsystem, Elevator elevatorSubsystem) {
        this.desiredLocation = desiredLocation;
        this.arm = armSubsystem;
        this.wrist = wristSubsystem;
        this.elevator = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        //Check to see the desired location has changed, and then choose the new desired positions
        //  if (desiredLocation != previousDesiredLocation){

          /*
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

            */

                switch (desiredLocation) {
                    case "CoralL1":
                        armDesiredRotation = ArmConstants.kCoralL1;
                        wristDesiredRotation = WristConstants.kCoralL1;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL1;
                    break;
                    case "CoralL2":
                        armDesiredRotation = ArmConstants.kCoralL2;
                        wristDesiredRotation = WristConstants.kCoralL2;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL2;
                    break;
                    case "CoralL3":
                        armDesiredRotation = ArmConstants.kCoralL3;
                        wristDesiredRotation = WristConstants.kCoralL3;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL3;
                    break;
                    case "CoralL4":
                        armDesiredRotation = ArmConstants.kCoralL4;
                        wristDesiredRotation = WristConstants.kCoralL4;
                        elevatorDesiredRotation = ElevatorConstants.kCoralL4;
                    break;
                    case "CoralGroundPickup":
                        armDesiredRotation = ArmConstants.kCoralLoadFloor;
                        wristDesiredRotation = WristConstants.kCoralLoadFloor;
                        elevatorDesiredRotation = ElevatorConstants.kCoralLoadFloor;
                    break;
                    case "CoralHumanPickup":
                        armDesiredRotation = ArmConstants.kCoralLoadHuman;
                        wristDesiredRotation = WristConstants.kCoralLoadHuman;
                        elevatorDesiredRotation = ElevatorConstants.kCoralLoadHuman;
                    break;
                    case "CoralStow":
                        armDesiredRotation = ArmConstants.kCoralStow;
                        wristDesiredRotation = WristConstants.kCoralStow;
                        elevatorDesiredRotation = ElevatorConstants.kCoralStow;
                    break;
                    default:
                        armDesiredRotation = arm.getArmRotation().getAsDouble();
                    break;
                }

                System.out.println("Arm POS: " + arm.getArmRotation().getAsDouble());
            //}
      //  }

        //This statement determines the starting point by looking at the arm position
        if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kRearSafe)) // We are in front of the robot, rotating past rearsafe
        {
            movement = movementState.FRONT_PAST_REAR_SAFE;


        } else if ((arm.getArmRotation().getAsDouble() > ArmConstants.kRearSafe) && (armDesiredRotation < ArmConstants.kFrontSafe)) //We are behind the robot, rotating past frontsafe
        {
            movement = movementState.REAR_PAST_FRONT_SAFE;


        } else if ((arm.getArmRotation().getAsDouble() < ArmConstants.kFrontSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the front of the robot, rotating middle
        {
            movement = movementState.FRONT_MIDDLE;


        } else if ((arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe) && (armDesiredRotation < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the rear of the robot, rotating middle
        {
            movement = movementState.REAR_MIDDLE;


        } else if ((arm.getArmRotation().getAsDouble() > ArmConstants.kFrontSafe) && (arm.getArmRotation().getAsDouble() < ArmConstants.kRearSafe) && (armDesiredRotation < ArmConstants.kFrontSafe))  //We are in the middle of the robot, rotating forward
        {
            movement = movementState.MIDDLE_PAST_FRONT_SAFE;


        } else if ((arm.getArmRotation().getAsDouble() > ArmConstants.kFrontSafe) && (arm.getArmRotation().getAsDouble() < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kIntakeSafe))  //We are in the middle of the robot
        {
            movement = movementState.MIDDLE_PAST_REAR_SAFE;  
        
        } else if ((arm.getArmRotation().getAsDouble() > ArmConstants.kFrontSafe) && (arm.getArmRotation().getAsDouble() < ArmConstants.kIntakeSafe) && (armDesiredRotation < ArmConstants.kIntakeSafe) && (armDesiredRotation > ArmConstants.kFrontSafe))  //We are in the middle of the robot
        {
            movement = movementState.MIDDLE_MIDDLE;  //For the rare situation where we are in the middle and tell it to go to the middle
        
        
        }else 
        {
            //We are in an unknown position.  Stop.
            movement = movementState.NONE;
        }

        previousDesiredLocation = desiredLocation;
        System.out.println("movement state: " + movement);
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
         }
    }
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
}
