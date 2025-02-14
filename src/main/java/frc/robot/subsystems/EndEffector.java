package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ArmConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.sim.ArmSim;

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

    private String GetEndEffectorMode() {
        return (m_endEffectorMode);
    }

    //=========================================================
    //==========================Public Command=================
    //=========================================================

    public Command setEndEffectorMode(String mode) {
        return run(
            () -> {this.setEndEffectorModeHere(mode);};

    }

    public Supplier<String> endEffectorMode() {
        return this::GetEndEffectorMode; 
    }

}
