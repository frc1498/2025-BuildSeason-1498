package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.CoralIntake;
import frc.robot.constants.EndEffectorConstants.endEffectorLocation;




public class Move {

    public CoralIntake intake;
    public Arm arm;
    public Wrist wrist;
    public Elevator elevator;

    public endEffectorLocation endEffectorLocation;

    public Move(Wrist wrist, Arm arm, CoralIntake intake, Elevator elevator) {
        this.intake = intake;
        this.arm = arm;
        this.wrist = wrist;
        this.elevator = elevator;
    }

    //==========================================================
    //=====================Configuration========================
    //==========================================================

    //==========================================================
    //======================Private=============================
    //==========================================================

    //==========================================================
    //=====================Commands=============================
    //==========================================================

    //Coral Intake Floor - Front to Front - second draft
    public Command intakeCoralFloorFrontToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.parallel(intake.intakeFloor(), elevator.elevatorCoralLoadFloor())).
        andThen(Commands.parallel(wrist.wristCoralLoadFloor(), arm.armCoralLoadFloor())).
        andThen(Commands.parallel(wrist.suck(),intake.rollerSuck())).until(wrist.isPartForwardGripper).
        andThen(wrist.stop()).
        andThen(Commands.parallel(arm.armCoralStow(), wrist.wristCoralStow(), intake.clearCoralIntake())).
        andThen(intake.intakeRaised()).
        andThen(intake.clearCoralIntake());
    }

    //Coral Intake Floor - Rear to Front - second draft
    public Command intakeCoralFloorRearToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(), wrist.wristCoralStow())).
        andThen(Commands.parallel(arm.armCoralStow(), elevator.elevatorCoralStow())). 
        andThen(Commands.parallel(arm.armCoralLoadFloor(),wrist.wristCoralLoadFloor())).
        andThen(wrist.suck(/*endEffectorLocation*/),intake.rollerSuck()).until(wrist.isPartForwardGripper).
        andThen(wrist.stop()).
        andThen(Commands.parallel(arm.armCoralStow(), wrist.wristCoralStow(), intake.clearCoralIntake())).
        andThen(intake.intakeRaised()).
        andThen(intake.clearCoralIntake());
    }

    /*
    //Intake Coral Human - Front to Rear - first draft
    public Command intakeCoralHumanFrontToRear(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(intake.rollerStop(), elevator.elevatorRearSafe(), arm.armCoralStow(), wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralLoadHuman(), wrist.wristCoralLoadHuman())).
        andThen(Commands.parallel(elevator.elevatorCoralLoadHuman(), wrist.suck(endEffectorLocation))).
        until(wrist.isPartForwardGripper).
        andThen(Commands.parallel(wrist.stop(),elevator.elevatorRearSafe()).
        andThen(Commands.parallel(arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(elevator.elevatorCoralStow()));
    }


    //Intake Coral Human - Rear to Rear - first draft
    public Command intakeCoralHumanRearToRear(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(elevator.elevatorRearSafe(), wrist.stop(), intake.rollerStop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralLoadHuman(), wrist.wristCoralLoadHuman())).
        andThen(elevator.elevatorCoralLoadHuman()).
        andThen(wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper).
        andThen(Commands.parallel(wrist.stop(),elevator.elevatorRearSafe())).
        andThen(Commands.parallel(arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(elevator.elevatorCoralStow());
    }
    */

    //Coral Stow - Front to Front - second draft
    public Command coralStowFrontToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.parallel(elevator.elevatorCoralStow(), arm.armCoralStow(), wrist.wristCoralStow())).
        andThen(intake.intakeRaised());
    }    

    //Coral Stow - Rear to Front - second draft
    public Command coralStowRearToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(arm.armCoralStow()).
        andThen(elevator.elevatorCoralStow()); 
    }    

    //Coral Score L4 - Front to Rear - second draft
    public Command coralL4FrontToRear() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralStow(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(Commands.parallel(arm.armCoralL4(), wrist.wristCoralL4()), elevator.elevatorCoralL4());
    }

    //Coral Score L4 - Rear to Rear - second draft
    public Command coralL4RearToRear() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).  
        andThen(Commands.parallel(elevator.elevatorCoralL4(), intake.intakeRaised(), arm.armCoralL4(), wrist.wristCoralL4()));
    }

    //Coral Score L3 - Front to Rear - second draft
    public Command coralL3FrontToRear() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralStow(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(arm.armAboveIntake45L3()).
        andThen(Commands.parallel(elevator.elevatorCoralL3(), arm.armCoralL3(), wrist.wristCoralL3()));
    }

    //Coral Score L3 - Rear to Rear - first draft
    public Command coralL3RearToRear() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.parallel(elevator.elevatorCoralL3(), arm.armCoralL3(), wrist.wristCoralL3()));
    }


    //Coral Score L2 - Front to Front - second draft
    public Command coralL2FrontToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.parallel(elevator.elevatorCoralL2(), arm.armCoralL2(), intake.intakeRaised(), wrist.wristCoralL2()))
        /*.andThen(Commands.parallel(intake.intakeRaised()))*/
        .withName("Coral L2 Front To Front");
    }

    //Coral Score L2 - Rear to Front - second draft
    public Command coralL2RearToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(arm.armCoralStow()).
        andThen(elevator.elevatorCoralL2(), arm.armCoralL2(), wrist.wristCoralL2());
    }

    //Coral Score L1 - Front to Front - second draft
    public Command coralL1FrontToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop())
        .andThen(Commands.parallel(elevator.elevatorCoralL1(), arm.armCoralL1(), intake.intakeRaised()))
        .andThen(wrist.wristCoralL1())
        .withName("Coral L1 Front To Front");
    }


    //Coral Score L1 - Rear to Front - second draft
    public Command coralL1RearToFront() {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(arm.armCoralStow()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL1(), wrist.wristCoralL1(), elevator.elevatorCoralL1()));
    }


    //Algae Remove L2 - Front to Front - second draft
    public Command goToRemoveAlgaeL2FrontToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.parallel(elevator.elevatorAlgaeL2(), wrist.wristAlgaeL2(),arm.armAlgaeL2())).
        andThen(wrist.spit(/*endEffectorLocation*/));
    }
    
    //Algae Remove L2 - Rear to Front - first draft
    public Command goToRemoveAlgaeL2RearToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(), wrist.wristCoralStow(), intake.intakeRaised())).
        andThen(arm.armCoralStow()).
        andThen(Commands.parallel(wrist.wristAlgaeL2(), arm.armAlgaeL2(), elevator.elevatorAlgaeL2())).
        andThen(wrist.spit(/*endEffectorLocation*/));
    }

    //Algae Remove L3 - Front to Front - second draft    
    public Command goToRemoveAlgaeL3FrontToRear(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()). 
        andThen(Commands.parallel(wrist.wristAlgaeL3(),arm.armAlgaeL3(),elevator.elevatorAlgaeL3())).
        andThen(wrist.spit(/*endEffectorLocation*/));
    }

    //Algae Remove L3 - Rear to Front - first draft
    public Command goToRemoveAlgaeL3RearToRear(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(wrist.stop(), intake.rollerStop()).
        andThen(Commands.deadline(elevator.elevatorRearSafe(), arm.armCoralLoadHuman(),wrist.wristCoralStow()).
        andThen(Commands.parallel( wrist.wristAlgaeL3(), arm.armAlgaeL3()))).
        andThen(Commands.parallel(elevator.elevatorAlgaeL3(), wrist.spit(/*endEffectorLocation*/)));
    }


    public Command clearClimb() {
        return Commands.parallel(elevator.elevatorRearSafe(), wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaisedForClimb(), arm.armClearClimb(), wrist.wristCoralL1())).
        andThen(elevator.elevatorCoralLoadFloor());
    }

    public Command intakeToRaisedForClimb() {
        return intake.intakeRaisedForClimb();
    }

    public Command clearCoralIntake() {
        return intake.clearCoralIntake();
    }

    /*
    public Command clearJams() {
        return Commands.parallel(elevator.elevatorRearSafe(), intake.intakeRaisedForClimb()).
        andThen(Commands.parallel(intake.clearCoralIntake(), arm.armCoralL1(), wrist.wristCoralStow())).
        andThen(wrist.clearWristCoral());
    }
    */

    public Command wristCoralRollerSpitFrontToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return wrist.spit(/*endEffectorLocation*/).
        until(wrist.isPartInGripper.negate()).
        andThen(wrist.stop()).
        andThen(arm.armCoralStow(), wrist.wristCoralStow(), elevator.elevatorCoralStow());
    }

    public Command wristCoralRollerSpitRearToFront(Supplier<endEffectorLocation> endEffectorLocation) {
        return wrist.spit(/*endEffectorLocation*/).
        until(wrist.isPartInGripper.negate()).
        andThen(wrist.stop()).
        andThen(arm.armCoralLoadHuman(), wrist.wristCoralStow(), elevator.elevatorRearSafe()).
        andThen(arm.armCoralStow()).
        andThen(elevator.elevatorCoralStow());
    }

    //======================================================
    //========================Triggers======================
    //======================================================
   

}
