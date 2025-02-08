package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase{

    public LimelightHelpers.PoseEstimate megaTag2;
    public DoubleSupplier robotHeading;

    public Pose2d testPose;
    public double testTimestamp;
    public RawFiducial[] fiducials;


    public Vision(DoubleSupplier robotHeading) {
        //Constructor.
        this.robotHeading = robotHeading;

        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 0);

        this.testPose = new Pose2d(8.5, 4.0, new Rotation2d(0.0)); //Center of the field

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

    private boolean coralStationInView() {
        return false;
    }

    private boolean bargeInView() {
        return false;
    }

    private boolean processorInView() {
        return false;
    }

    private boolean reefInView() {
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
    public Trigger coralStationInView = new Trigger(this::coralStationInView);
    public Trigger bargeInView = new Trigger(this::bargeInView);
    public Trigger processorInView = new Trigger(this::processorInView);
    public Trigger reefInView = new Trigger(this::reefInView);

    public Command addMegaTag2(Supplier<CommandSwerveDrivetrain> drivetrain) {
        return runOnce(
            () -> {
                testTimestamp = Utils.getCurrentTimeSeconds();
                drivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                drivetrain.get().addVisionMeasurement(testPose, testTimestamp);
            }
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

        //Get the raw fiducials.
        fiducials = LimelightHelpers.getRawFiducials(VisionConstants.kLimelightName);
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            double distToCamera = fiducial.distToCamera;  // Distance to camera
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
