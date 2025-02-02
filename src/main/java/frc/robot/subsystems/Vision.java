package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase{

    public LimelightHelpers.PoseEstimate megaTag2;
    public DoubleSupplier robotHeading;

    public Vision(DoubleSupplier robotHeading) {
        //Constructor.
        this.robotHeading = robotHeading;
    }

    private void setLimelightRobotPosition() {
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.kLimelightName,
            VisionConstants.kLimelightForwardOffset,
            VisionConstants.kLimelightSideOffset,
            VisionConstants.kLimelightUpOffset,
            VisionConstants.kLimelightRollOffset,
            VisionConstants.kLimelightPitchOffset,
            VisionConstants.kLimelightYawOffset);
    }

    private boolean isPoseValid() {
        return false;
    }

    public Trigger addLimelightPose = new Trigger(this::isPoseValid);

    public Command addMegaTag2(Supplier<CommandSwerveDrivetrain> drivetrain) {
        return runOnce(
            () -> drivetrain.get().addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds)
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
    }    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, robotHeading.getAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
        megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
