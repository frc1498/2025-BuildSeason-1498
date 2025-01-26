package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase{

    TalonFX ArmRotate;
    CANcoder armRotateEncoder;
    PositionVoltage RotateControl;



    public Arm() {
        //Constructor.
        ArmRotate = new TalonFX(ArmConstants.kArmRotateCANID, "canivore");
        armRotateEncoder = new CANcoder(ArmConstants.kEncoderCANID,"canivore");
        RotateControl = new PositionVoltage(ArmConstants.kStowCoral);

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
    public Command GoToStow(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kStowCoral);}
        );
        
    }
    public Command GoToL1(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kL1Coral);}
        );
        
    }
    public Command GoToLoad(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kLoadCoral);}
        );
        
    }
    public Command GoToL2(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kL2Coral);}
        );
        
    }
    public Command GoToL3(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kL3Coral);}
        );
        
    }
    public Command GoToL4(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kL4Coral);}
        );
        
    }
    public Command GoToBarge(){
        return run(
            () -> {this.GoToPosition(ArmConstants.kBargeCoral);}
        );
        
    }
   public Trigger IsArmStow = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kStowCoral);
   });
   
   public Trigger IsArmLoad = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kLoadCoral);
   });

   public Trigger IsArmL1 = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kL1Coral);
   });

   public Trigger IsArmL2 = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kL2Coral);
   });
   
   public Trigger IsArmL3 = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kL3Coral);
   });

   public Trigger IsArmL4 = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kL4Coral);
   });

   public Trigger IsArmBarge = new Trigger(() ->{
    return this.IsArmAtPosition(ArmConstants.kBargeCoral);
   });


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
