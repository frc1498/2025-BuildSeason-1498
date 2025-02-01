package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

public class LED extends SubsystemBase{

    CANdle lighting;
    
    public LED() {
        //Constructor.
        lighting = new CANdle(LEDConstants.kCANdleCANID, "Canivore");
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
