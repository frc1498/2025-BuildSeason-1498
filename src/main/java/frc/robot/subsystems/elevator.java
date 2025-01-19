package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.elevatorConstants;

public class elevator extends SubsystemBase {
    TalonFX elevatorDriveFront = new TalonFX(elevatorConstants.kElevatorDriveFrontCANID, "canivore");
    TalonFX elevatorDriveBack = new TalonFX(elevatorConstants.kElevatorDriveBackCANID, "canivore");

    private void elevatorDriveToPosition(double position) {

    }
    
    public Command elevatorHome() {
        return run(
            () -> {this.elevatorDriveToPosition(elevatorConstants.kHome);}
            ).until(this.isElevatorHome());
    }

    public Trigger isElevatorHome() {
        return elevatorCurrentPosition == elevatorConstants.kHome;
    }
}