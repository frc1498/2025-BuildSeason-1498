package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.WristConfig;
import frc.robot.constants.WristConstants;
import frc.robot.sim.ElleySim;

public class Wrist extends SubsystemBase{
    TalonFX wristRotate;
    CANcoder wristRotateEncoder;
    PositionVoltage rotateControl;

    public Wrist(WristConfig config) {
        //Constructor.
        wristRotate = new TalonFX(WristConstants.kWristRotateCANID, "canivore");
        wristRotateEncoder = new CANcoder(WristConstants.kWristEncoderCANID,"canivore");
        rotateControl = new PositionVoltage(WristConstants.kStow);

        wristRotate.getConfigurator().apply(config.WristConfig);
        this.config = config;
        sim = new WristSim(config, WristDriveSim);
    }
    
    private void GoToPosition(double position) {
            wristRotate.setControl(rotateControl.withPosition(position));
    }

    private boolean IsWristAtPosition(double position) {
            return ((position-WristConstants.kDeadband) <= GetWristPosition()) && ((position+WristConstants.kDeadband) >= GetWristPosition());
    }

    private double GetWristPosition(){
            return wristRotate.getPosition().getValueAsDouble();       
    }

    public Command CommandGoToPosition(double position){
            return run(() -> {this.GoToPosition(position);});
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
