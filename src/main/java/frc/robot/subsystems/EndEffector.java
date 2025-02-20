package frc.robot.subsystems;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ElevatorConstants;

public class EndEffector extends SubsystemBase{

    public String m_endEffectorMode = "Algae";

     public EndEffector() {
        
    }

    //========================================================
    //======================= Private ========================
    //========================================================
    private void setEndEffectorModeHere(String mode) {
        if (ElevatorConstants.kElevatorPrint){
            System.out.println("=============private setEndEffectorModeHere===============");
            System.out.println("Mode:"+mode);    
        }
        this.m_endEffectorMode = mode;
    }

     Boolean GetEndEffectorMode() {
        if (ElevatorConstants.kElevatorPrintTrigger){
            System.out.println("=============private getEndEffectorMode===============");
            System.out.println("End Effector Mode:" + m_endEffectorMode);
        }

        return (m_endEffectorMode == "Algae");
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {
        if (ElevatorConstants.kElevatorPrint){
            System.out.println("=============Command setEndEffectorMode===============");
            System.out.println("End Effector Mode:" + mode);
        }

        return run(() -> {this.setEndEffectorModeHere(mode);});
    }

    public Command whatIsEndEffectorMode() {

        return run(() -> {this.GetEndEffectorMode();});
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
