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
    public Command intakeCoralFloor(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(intake.intakeFloor(), elevator.toIntakeSafe()).
        andThen(Commands.parallel(arm.armCoralLoadFloor(),wrist.wristCoralLoadFloor())).
        andThen(elevator.elevatorCoralLoadFloor()).
        andThen(Commands.parallel(intake.rollerSuck(),wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper)).
        andThen(Commands.parallel(intake.clearCoralIntake(),wrist.stop(),elevator.toIntakeSafe())).
        andThen(Commands.parallel(intake.clearCoralIntake(),arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()).
        andThen(intake.rollerStop()));
    }

    public Command intakeCoralFloorBetter(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(intake.intakeFloor(), Commands.sequence(elevator.toIntakeSafe(),Commands.parallel(arm.armCoralLoadFloorBetter(),wrist.wristCoralLoadFloorBetterInitial()))).
        andThen(elevator.elevatorCoralLoadFloor()).
        andThen(wrist.wristCoralLoadFloorBetterFinal()).
        andThen(Commands.parallel(intake.rollerSuck(),wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper)).
        andThen(Commands.parallel(wrist.stop(),intake.rollerStop())).
        andThen(Commands.parallel(wrist.wristCoralLoadFloorBetterInitial()),elevator.toIntakeSafe(),intake.clearCoralIntake()).
        andThen(Commands.parallel(intake.intakeRaised(),wrist.wristCoralStow(),arm.armCoralStow())).
        andThen(elevator.elevatorCoralStow(),intake.rollerStop());
    }
/*
    public Command intakeCoralFloorBetter(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(intake.intakeFloor(), elevator.toIntakeSafe()).
        andThen(Commands.parallel(arm.armCoralLoadFloorBetter(),wrist.wristCoralLoadFloorBetterInitial())).
        andThen(elevator.elevatorCoralLoadFloor()).
        andThen(wrist.wristCoralLoadFloorBetterFinal()).
        andThen(Commands.parallel(intake.rollerSuck(),wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper)).
        andThen(Commands.parallel(wrist.stop())).
        andThen(Commands.parallel(wrist.wristCoralLoadFloorBetterInitial()),elevator.toIntakeSafe()).
        andThen(Commands.parallel(intake.intakeRaised(),wrist.wristCoralStow(),arm.armCoralStow())).
        andThen(elevator.elevatorCoralStow(),intake.clearCoralIntake());
    }
*/

/*
    .
        andThen(Commands.parallel(intake.rollerSuck(),wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper)).
        andThen(Commands.parallel(intake.clearCoralIntake(),wrist.stop(),elevator.toIntakeSafe())).
        andThen(Commands.parallel(intake.clearCoralIntake(),arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()).
        andThen(intake.rollerStop())
*/

    /*
    public Command intakeCoralFloorFast(Supplier<endEffectorLocation> endEffectorLocation) {
        return Commands.parallel(intake.intakeFloor(), elevator.toIntakeSafe()).
        andThen(Commands.parallel(arm.armCoralLoadFloor(),wrist.wristCoralLoadFloor())).
        andThen(elevator.elevatorCoralLoadFloor()).
        andThen(Commands.parallel(intake.rollerSuck(),wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper)).
        andThen(Commands.parallel(intake.rollerStop(),wrist.stop())).
        andThen(Commands.parallel(elevator.elevatorCoralStowFast(), wrist.wristCoralStow()).
        andThen(arm.armCoralStow()).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()));
    }
    */

    public Command coralStow() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralStow(), wrist.wristCoralStow())).
        andThen(Commands.parallel(elevator.elevatorCoralStow(), wrist.stop(), intake.rollerStop()));
    }    

    public Command coralL4() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL4(), wrist.wristCoralL4())).
        andThen(elevator.elevatorCoralL4());
    }

    public Command coralL3() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL3(), wrist.wristCoralL3())).
        andThen(elevator.elevatorCoralL3());
    }

    public Command coralL2() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL2(), wrist.wristCoralL2())).
        andThen(elevator.elevatorCoralL2());
    }

    public Command wristCoralRollerSpit(Supplier<endEffectorLocation> endEffectorLocation) {
        return wrist.spit(endEffectorLocation);
    }

    public Command wristCoralRollerStop() {
        return wrist.stop();
    }

    public Command intakeCoralHuman(Supplier<endEffectorLocation> endEffectorLocation) {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralLoadHuman(), wrist.wristCoralLoadHuman())).
        andThen(elevator.elevatorCoralLoadHuman()).
        andThen(wrist.suck(endEffectorLocation)).until(wrist.isPartForwardGripper).
        andThen(wrist.stop()).
        andThen(elevator.toIntakeSafe()).
        andThen(Commands.parallel(arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()));
    }

    public Command coralL1() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL1(), wrist.wristCoralL1())).
        andThen(elevator.elevatorCoralL1());
    }

    public Command clearClimb() {
        return Commands.parallel(elevator.toIntakeSafe(),wrist.stop()).
        andThen(Commands.parallel(intake.intakeRaisedForClimb(), arm.armClearClimb(), wrist.wristCoralL1())).
        andThen(elevator.elevatorCoralLoadFloor());
    }

    public Command intakeToRaisedForClimb() {
        return intake.intakeRaisedForClimb();
    }

    public Command clearCoralIntake() {
        return intake.clearCoralIntake();
    }

    public Command clearJams() {
        return Commands.parallel(elevator.toIntakeSafe(), intake.intakeRaisedForClimb()).
        andThen(Commands.parallel(intake.clearCoralIntake(), arm.armCoralLoadHuman(), wrist.wristCoralLoadHuman())).
        andThen(wrist.clearWristCoral()).
        andThen(Commands.parallel(arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()));
    }


    public Command goToRemoveAlgaeL2(Supplier<endEffectorLocation> endEffectorLocation) {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(wrist.wristAlgaeL2(),arm.armAlgaeL2())).
        andThen(elevator.elevatorAlgaeL2(), wrist.spit(endEffectorLocation));
    }

    
    public Command goToRemoveAlgaeL3(Supplier<endEffectorLocation> endEffectorLocation) {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(wrist.wristAlgaeL3(),arm.armAlgaeL3())).
        andThen(elevator.elevatorAlgaeL3(), wrist.spit(endEffectorLocation));
    }

    //======================================================
    //========================Triggers======================
    //======================================================
   

}
