package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;

public class EndEffector {
    ElevatorConfig elevatorConfig;
    Elevator elevator;
    ArmConfig armConfig;
    Arm arm;
    WristConfig wristConfig;
    Wrist wrist;

    public EndEffector() {
        elevatorConfig = new ElevatorConfig();
        elevator = new Elevator(elevatorConfig);
        armConfig = new ArmConfig();
        arm = new Arm(armConfig);
        wristConfig = new WristConfig();
        wrist = new Wrist(wristConfig);
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

    public Command toCoralSuck() {
        return elevator.elevatorCoralLoadFloor().until(elevator.isElevatorCoralLoadFloor)
        .andThen(arm.armCoralLoadFloor()).until(arm.isArmCoralLoadFloor)
        .andThen(wrist.wristCoralLoadFloor()).until(wrist.isWristCoralLoadFloor)
        .andThen(wrist.suck());
    }

    public Command toCoralStow() {
        return 
            elevator.elevatorCoralStow().until(elevator.isElevatorCoralStow)
            .andThen(arm.armCoralStow()).until(arm.isArmCoralStow)
            .andThen(wrist.wristCoralStow()).until(wrist.isWristCoralStow);
        
    }

    public Command toCoralL1() {
        return 
            elevator.elevatorCoralL1().until(elevator.isElevatorCoralL1)
            .andThen(arm.armCoralL1()).until(arm.isArmCoralL1)
            .andThen(wrist.wristCoralL1()).until(wrist.isWristCoralL1);
        
    }

    public Command toCoralL2() {
        return 
            elevator.elevatorCoralL2().until(elevator.isElevatorCoralL2)
            .andThen(arm.armCoralL2()).until(arm.isArmCoralL2)
            .andThen(wrist.wristCoralL2()).until(wrist.isWristCoralL2);
        
    }

    public Command toCoralL3() {
        return 
            elevator.elevatorCoralL3().until(elevator.isElevatorCoralL3)
            .andThen(arm.armCoralL3()).until(arm.isArmCoralL3)
            .andThen(wrist.wristCoralL3()).until(wrist.isWristCoralL3);
        
    }

    
    //======================================================
    //========================Triggers======================
    //======================================================


}
