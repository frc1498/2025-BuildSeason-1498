package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase{

    TalonFX ArmRotate;
    CANcoder armRotateEncoder;
    PositionVoltage RotateControl;



    public Arm() {
        //Constructor.
        ArmRotate = new TalonFX(ArmConstants.kArmRotateCANID, "canivore");
        armRotateEncoder = new CANcoder(ArmConstants.kEncoderCANID,"canivore");
        RotateControl = new PositionVoltage(ArmConstants.kStow);

    }
    private void GoToPosition(double position) {
            ArmRotate.setControl(RotateControl.withPosition(position));

    }
    private boolean IsArmAtPosition(double position) {
            return ((position-ArmConstants.kDeadband) <= GetArmPosition()) && ((position+ArmConstants.kDeadband) >= GetArmPosition());
    }
    private double GetArmPosition(){
            return ArmRotate.getPosition().getValueAsDouble();
            
    }

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
