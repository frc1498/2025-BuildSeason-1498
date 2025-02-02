package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ArmConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;

public class EndEffector extends SubsystemBase{
    ElevatorConfig elevatorConfig;
    elevator elevator;
    ArmConfig armConfig;
    Arm arm;
    WristConfig wristConfig;
    Wrist wrist;

    public EndEffector() {
        elevatorConfig = new ElevatorConfig();
        elevator = new elevator(elevatorConfig);
        armConfig = new ArmConfig();
        arm = new Arm(armConfig);
        wristConfig = new WristConfig();
        wrist = new Wrist(wristConfig);
    }

    public Command startToCoralStow() {
        return run(() -> {
            elevator.elevatorCoralStow().until(elevator.isElevatorCoralStow)
            .andThen(arm.armCoralStow()).until(arm.isArmCoralStow)
            .andThen(wrist.wristCoralStow()).until(wrist.isWristCoralStow);
        });
    }
}
