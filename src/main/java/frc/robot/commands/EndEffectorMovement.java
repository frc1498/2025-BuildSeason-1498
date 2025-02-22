package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ArmConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;

public class EndEffectorMovement {

    public Elevator elevator;
    public Arm arm;
    public Wrist wrist;
    public EndEffector endEffector;

    double armDesiredRotation2 = 0;
    double wristDesiredRotation2 = 0;
    double elevatorDesiredRotation2 = 0;

    String previousDesiredLocation = "";

    public EndEffectorMovement(Arm arm, Wrist wrist, Elevator elevator, EndEffector endEffector) {
        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
        this.endEffector = endEffector;
    }

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
//====================Goto Coral Locations==============================
    public Command toCoralL1(){

        return Commands.parallel(new EndEffectorScheduling("CoralL1", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toCoralL1"));    
    }
    
    public Command toCoralL2() {

        return Commands.parallel(new EndEffectorScheduling("CoralL2", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toCoralL2"));   
    }

    public Command toCoralL3() {

        return Commands.parallel(new EndEffectorScheduling("CoralL3", arm, wrist, elevator),
            Commands.print("EndEffectorCommand:toCoralL3"));
    }

    public Command toCoralL4() {

        return Commands.parallel(new EndEffectorScheduling("CoralL4", arm, wrist, elevator),
            Commands.print("EndEffectorCommand:toCoralL4"));
    }

    public Command toCoralGroundPickup() {

        return Commands.parallel(new EndEffectorScheduling("CoralGroundPickup", arm, wrist, elevator),
            Commands.print("EndEffectorCommand:toCoralGroundPickup"));
    }

    public Command toCoralHumanPickup() {

        return Commands.parallel(new EndEffectorScheduling("CoralHumanPickup", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toCoralHumanPickup"));
    }

    public Command toCoralStow() {

        return Commands.parallel(new EndEffectorScheduling("CoralStow", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toCoralStow"));
    }

    //==================Goto Alage Locations================================
    public Command toAlgaePickup() {      

        return Commands.parallel(new EndEffectorScheduling("AlgaePickup", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toAlgaePickup"));
    }

    public Command toAlgaeScoreBarge() {

        return Commands.parallel(new EndEffectorScheduling("AlgaeScoreBarge", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toAlgeaScoreBarge"));
    }
    
    public Command toAlgaeL2() {

        return Commands.parallel(new EndEffectorScheduling("AlgaeL2", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toAlgaeL2"));
    }
    
    public Command toAlgaeL3() {

        return Commands.parallel(new EndEffectorScheduling("AlgaeL3", arm, wrist, elevator),
        Commands.print("EndEffectorCommand:toAlgaeL3"));
    }
    
    public Command toAlgaeProcessor() {

        return Commands.parallel(new EndEffectorScheduling("AlgaeProcessor", arm, wrist, elevator),
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
