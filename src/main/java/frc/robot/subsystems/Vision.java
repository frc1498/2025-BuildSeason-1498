package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        SmartDashboard.putData("Vision", this);
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

    private double getRobotHeading() {
        return this.robotHeading.getAsDouble();
    }

    private void takeSnapshot() {
        //In the future, this should generate a unique name for the snapshot.
        LimelightHelpers.takeSnapshot(VisionConstants.kLimelightName, "Snappy");
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
        builder.addDoubleProperty("Robot Heading", this::getRobotHeading, null);
    }    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, robotHeading.getAsDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
        //THIS LINE IS EXTREMELY IMPORTANT
        megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
