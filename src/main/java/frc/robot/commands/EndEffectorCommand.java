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
        MIDDLE_MIDDLE,
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

    public EndEffectorCommand(EndEffector endEffector) {
        elevatorConfig = new ElevatorConfig();
        elevator = new Elevator(elevatorConfig);
        armConfig = new ArmConfig();
        arm = new Arm(armConfig);
        wristConfig = new WristConfig();
        wrist = new Wrist(wristConfig);
        this.endEffector = endEffector;

        movement = movementState.NONE;

    }

    //==========================================================
    //=====================Configuration========================
    //==========================================================




    
    //==========================================================
    //======================Private=============================
    //==========================================================

    private Boolean isEndEffectorAtPosition(String desiredLocation) {
        System.out.println("EndEffectorCommand:isEndEffectorAtPosition was called");

        //This section checks to see if we have changed desired locations.  If so, save new desired locations.

        if (desiredLocation != previousDesiredLocation){
            if (endEffector.isModeAlgae.getAsBoolean()){     //Are we Algae mode     
                switch (desiredLocation) {
                    case "L1orProcessor":
                        armDesiredRotation2 = ArmConstants.kAlgaeProcessor;
                        wristDesiredRotation2 = WristConstants.kAlgaeProcessor;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeProcessor;
                    break;
                    case "AlgaeL2":
                        armDesiredRotation2 = ArmConstants.kAlgaeL2;
                        wristDesiredRotation2 = WristConstants.kAlgaeL2;
                        elevatorDesiredRotation2 = ElevatorConstants.kAlgaeL2;
                    break;
                    case "AlgaeL3":
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
                    case "CoralL1":
                        armDesiredRotation2 = ArmConstants.kCoralL1;
                        wristDesiredRotation2 = WristConstants.kCoralL1;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL1;
                    break;
                    case "CoralL2":
                        armDesiredRotation2 = ArmConstants.kCoralL2;
                        wristDesiredRotation2 = WristConstants.kCoralL2;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL2;
                    break;
                    case "CoralL3":
                        armDesiredRotation2 = ArmConstants.kCoralL3;
                        wristDesiredRotation2 = WristConstants.kCoralL3;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL3;
                    break;
                    case "CoralL4":
                        armDesiredRotation2 = ArmConstants.kCoralL4;
                        wristDesiredRotation2 = WristConstants.kCoralL4;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralL4;
                    break;
                    case "CoralGroundPickup":
                        armDesiredRotation2 = ArmConstants.kCoralLoadFloor;
                        wristDesiredRotation2 = WristConstants.kCoralLoadFloor;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralLoadFloor;
                    break;
                    case "CoralHumanPickup":
                        armDesiredRotation2 = ArmConstants.kCoralLoadHuman;
                        wristDesiredRotation2 = WristConstants.kCoralLoadHuman;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralLoadHuman;
                    break;
                    case "CoralStow":
                        armDesiredRotation2 = ArmConstants.kCoralStow;
                        wristDesiredRotation2 = WristConstants.kCoralStow;
                        elevatorDesiredRotation2 = ElevatorConstants.kCoralStow;
                    break;
                }
            }
        }


         //   System.out.println("=========================End Effector In Position?=====================");
         //   System.out.println("Algae Mode:" + endEffector.isModeAlgae.getAsBoolean());
         //   System.out.println("End Effector Desired Location: " + desiredLocation);
         //   System.out.println("wrist Desired Rotation:" + wristDesiredRotation2);
         //   System.out.println("arm Desired Rotation:" + armDesiredRotation2);
         //   System.out.println("arm Desired Rotation:" + elevatorDesiredRotation2);
         //   System.out.println("wrist Actual Rotation:" + wrist.getWristRotation().getAsDouble());
         //   System.out.println("arm Actual Rotation:" + arm.getArmRotation().getAsDouble());
         //   System.out.println("elevator Actual Rotation:" + elevator.getElevatorRotation().getAsDouble());

        //Check to see if we are at any of the locations
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


        //    System.out.println("=========================Command End Effector moveEndEffector=====================");
        //    System.out.println("Algae Mode:" + endEffector.isModeAlgae.getAsBoolean());
        //    System.out.println("End Effector Desired Location: " + desiredLocation);
        //    System.out.println("wrist Desired Rotation:" + wristDesiredRotation);
        //    System.out.println("arm Desired Rotation:" + armDesiredRotation);
        //    System.out.println("arm Desired Rotation:" + elevatorDesiredRotation);
        //    System.out.println("wrist Actual Rotation:" + wrist.getWristRotation().getAsDouble());
        //    System.out.println("arm Actual Rotation:" + arm.getArmRotation().getAsDouble());
        //    System.out.println("elevator Actual Rotation:" + elevator.getElevatorRotation().getAsDouble());
        

        previousDesiredLocation = desiredLocation;

        switch (movement) {
            case FRONT_PAST_REAR_SAFE:  
                return 
                    Commands.parallel((this.toFrontSafe()
                    .andThen(this.toRearSafe())
                    .andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            case REAR_PAST_FRONT_SAFE:
                return 
                    Commands.parallel((this.toRearSafe()
                    .andThen(this.toFrontSafe())
                    .andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            case FRONT_MIDDLE:
                return 
                    Commands.parallel((this.toFrontSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector:"+ movement));
            case REAR_MIDDLE:
                return 
                    Commands.parallel((this.toRearSafe().
                    andThen(this.toIntakeSafe()).andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation),
                    Commands.print("(arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe): " + (arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe)),
                    Commands.print("(armDesiredRotation < ArmConstants.kIntakeSafe): " + (armDesiredRotation < ArmConstants.kIntakeSafe) ),
                    Commands.print("(armDesiredRotation > ArmConstants.kFrontSafe): " + (armDesiredRotation > ArmConstants.kFrontSafe)));
           
           
            case MIDDLE_PAST_FRONT_SAFE:
                return 
                    Commands.parallel((this.toFrontSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement));
            case MIDDLE_PAST_REAR_SAFE:          
                return 
                    Commands.parallel((this.toIntakeSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation),
                    Commands.print("(arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe): " + (arm.getArmRotation().getAsDouble() > ArmConstants.kIntakeSafe)),
                    Commands.print("(armDesiredRotation < ArmConstants.kIntakeSafe): " + (armDesiredRotation < ArmConstants.kIntakeSafe) ),
                    Commands.print("(armDesiredRotation > ArmConstants.kFrontSafe): " + (armDesiredRotation > ArmConstants.kFrontSafe)));
            case MIDDLE_MIDDLE:          
                return 
                    Commands.parallel((this.toMiddleSafe().
                    andThen(this.toDesired(wristDesiredRotation, armDesiredRotation, elevatorDesiredRotation))),
                    Commands.print("EndEffectorCommand:moveEndEffector: "+ movement),
                    Commands.print("Arm Location: " + arm.getArmRotation().getAsDouble()),
                    Commands.print("Arm Desired Rotation: " + armDesiredRotation),
                    Commands.print("General Desired location: " + desiredLocation));
            default:                     
                return //This sets everything to where it currently is because something is wrong.....
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

      //    System.out.println("=========================Command End Effector moveEndEffector=====================");
        //    System.out.println("Algae Mode:" + endEffector.isModeAlgae.getAsBoolean());
        //    System.out.println("End Effector Desired Location: " + desiredLocation);
        //    System.out.println("wrist Desired Rotation:" + wristDesiredRotation);
        //    System.out.println("arm Desired Rotation:" + armDesiredRotation);
        //    System.out.println("arm Desired Rotation:" + elevatorDesiredRotation);
        //    System.out.println("wrist Actual Rotation:" + wrist.getWristRotation().getAsDouble());
        //    System.out.println("arm Actual Rotation:" + arm.getArmRotation().getAsDouble());
        //    System.out.println("elevator Actual Rotation:" + elevator.getElevatorRotation().getAsDouble());
        







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

//====================Goto Coral Locations==============================
    public Command toCoralL1(){

        return Commands.parallel(this.moveEndEffector("CoralL1"),
        Commands.print("EndEffectorCommand:toCoralL1"));    
    }
    
    public Command toCoralL2() {

        return Commands.parallel(this.moveEndEffector("CoralL2"),
        Commands.print("EndEffectorCommand:toCoralL2"));   
    }

    public Command toCoralL3() {

        return Commands.parallel(this.moveEndEffector("CoralL3"),
            Commands.print("EndEffectorCommand:toCoralL3"));
    }

    public Command toCoralL4() {

        return Commands.parallel(this.moveEndEffector("CoralL4"),
            Commands.print("EndEffectorCommand:toCoralL4"));
    }

    public Command toCoralGroundPickup() {

        return Commands.parallel(this.moveEndEffector("CoralGroundPickup"),
            Commands.print("EndEffectorCommand:toCoralGroundPickup"));
    }

    public Command toCoralHumanPickup() {

        return Commands.parallel(this.moveEndEffector("CoralHumanPickup"),
        Commands.print("EndEffectorCommand:toCoralHumanPickup"));
    }

    public Command toCoralStow() {

        return Commands.parallel(this.moveEndEffector("CoralStow"),
        Commands.print("EndEffectorCommand:toCoralStow"));
    }

    //==================Goto Alage Locations================================
    public Command toAlgaePickup() {      

        return Commands.parallel(this.moveEndEffector("AlgaePickup"),
        Commands.print("EndEffectorCommand:toAlgaePickup"));
    }

    public Command toAlgaeScoreBarge() {

        return Commands.parallel(this.moveEndEffector("AlgaeScoreBarge"),
        Commands.print("EndEffectorCommand:toAlgeaScoreBarge"));
    }
    
    public Command toAlgaeL2() {

        return Commands.parallel(this.moveEndEffector("AlgaeL2"),
        Commands.print("EndEffectorCommand:toAlgaeL2"));
    }
    
    public Command toAlgaeL3() {

        return Commands.parallel(this.moveEndEffector("AlgaeL3"),
        Commands.print("EndEffectorCommand:toAlgaeL3"));
    }
    
    public Command toAlgaeProcessor() {

        return Commands.parallel(this.moveEndEffector("AlgaeProcessor"),
        Commands.print("EndEffectorCommand:toAlgaeProcessor"));
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
