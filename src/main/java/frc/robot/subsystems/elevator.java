package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    TalonFX elevatorDriveFront = new TalonFX(ElevatorConstants.kElevatorDriveFrontCANID, "canivore");
    TalonFX elevatorDriveBack = new TalonFX(ElevatorConstants.kElevatorDriveBackCANID, "canivore");

    public Elevator() {
        //Constructor.
    }

    private void elevatorDriveToPosition(double position) {

    }
    
    public Command elevatorHome() {
        return run(
            () -> {this.elevatorDriveToPosition(ElevatorConstants.kHome);}
            ).until(this.isElevatorHome);
    }

    public final Trigger isElevatorHome = new Trigger(() -> true);

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}