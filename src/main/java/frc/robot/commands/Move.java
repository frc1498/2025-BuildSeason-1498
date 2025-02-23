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
        andThen(Commands.parallel(intake.rollerStop(),wrist.stop())).
        andThen(elevator.toIntakeSafe()).
        andThen(Commands.parallel(arm.armCoralStow(),wrist.wristCoralStow())).
        andThen(Commands.parallel(intake.intakeRaised(),elevator.elevatorCoralStow()));
    }

    public Command coralStow() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralStow(), wrist.wristCoralStow())).
        andThen(Commands.parallel(elevator.elevatorCoralStow(), wrist.stop(), intake.rollerStop()));
    }    

    public Command coralL4() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL4(), wrist.wristCoralL4())).
        andThen(elevator.elevatorCoralL4());
    }

    public Command coralL3() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL3(), wrist.wristCoralL3())).
        andThen(elevator.elevatorCoralL3());
    }

    public Command coralL2() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL2(), wrist.wristCoralL2())).
        andThen(elevator.elevatorCoralL2());
    }

    public Command wristCoralRollerSpit(Supplier<endEffectorLocation> endEffectorLocation) {
        return wrist.spit(endEffectorLocation);
    }

    public Command wristCoralRollerStop() {
        return wrist.stop();
    }



    public Command intakeCoralHuman() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralLoadHuman(), wrist.wristCoralLoadHuman())).
        andThen(elevator.elevatorCoralLoadHuman());
    }

    public Command coralL1() {
        return elevator.toIntakeSafe().
        andThen(Commands.parallel(intake.intakeRaised(), arm.armCoralL1(), wrist.wristCoralL1())).
        andThen(elevator.elevatorCoralL1());
    }    

    //======================================================
    //========================Triggers======================
    //======================================================
   

}
