package frc.robot.subsystems;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.EndEffectorConstants.endEffectorLocation;

public class EndEffector extends SubsystemBase{

    public String m_endEffectorMode = "Coral";
    public endEffectorLocation m_endEffectorLocation = endEffectorLocation.NONE;

     public EndEffector() {
        SmartDashboard.putData("End Effector", this);
        
    }

    //========================================================
    //======================= Private ========================
    //========================================================
    private void setEndEffectorModeHere(String mode) {
        this.m_endEffectorMode = mode;
    }

    private Boolean GetEndEffectorModeHere() {

        return (this.m_endEffectorMode == "Algae");
    }

    private void setEndEffectorLocationHere(endEffectorLocation location) {
        this.m_endEffectorLocation = location;
    }

    private endEffectorLocation GetEndEffectorLocationHere() {

        return (this.m_endEffectorLocation);
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {

        return runOnce(() -> {this.setEndEffectorModeHere(mode);});
    }

    public Command whatIsEndEffectorMode() {

        return runOnce(() -> {this.GetEndEffectorModeHere();});
    }

    public Command setEndEffectorLocation(Supplier<endEffectorLocation> mode) {

        return runOnce(() -> {this.setEndEffectorLocationHere(mode.get());});
    }

    public Supplier<endEffectorLocation> whatIsEndEffectorLocation() {

        return this::GetEndEffectorLocationHere;
    }


    /*
    public Supplier<String> endEffectorMode() {
        return this::GetEndEffectorMode; 
    }
    */

    //======================================================
    //===================Triggers===========================
    //======================================================
    public final Trigger isModeAlgae = new Trigger(() -> {return this.GetEndEffectorModeHere();});

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addStringProperty("Current End Effector Location", () -> {return this.GetEndEffectorLocationHere().toString();}, null);

    }

}
