package frc.robot.subsystems;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase{

    public String m_endEffectorMode = "Coral";

     public EndEffector() {
        
    }

    //========================================================
    //======================= Private ========================
    //========================================================
    private void setEndEffectorModeHere(String mode) {

        System.out.println("EndEffector:setEndEffectorModeHere:  " + mode);    
        
        this.m_endEffectorMode = mode;
    }

     Boolean GetEndEffectorMode() {

        return (m_endEffectorMode == "Algae");
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {

        return runOnce(() -> {this.setEndEffectorModeHere(mode);});
    }

    public Command whatIsEndEffectorMode() {

        return runOnce(() -> {this.GetEndEffectorMode();});
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
