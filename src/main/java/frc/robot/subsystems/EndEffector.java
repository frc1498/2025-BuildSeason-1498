package frc.robot.subsystems;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EndEffector extends SubsystemBase{

    public String m_endEffectorMode = "Algae";

     public EndEffector() {
        
    }

    //========================================================
    //======================= Private ========================
    //========================================================
    private void setEndEffectorModeHere(String mode) {
        this.m_endEffectorMode = mode;
    }

    private Boolean GetEndEffectorMode() {
        return (m_endEffectorMode == "Algae");
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {
        return run(() -> {this.setEndEffectorModeHere(mode);});
    }

    /*
    public Supplier<String> endEffectorMode() {
        return this::GetEndEffectorMode; 
    }
    */

    //======================================================
    //===================Triggers===========================
    //======================================================
    public final Trigger isModeAlgae = new Trigger(() -> {return this.GetEndEffectorMode();});


}
