package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase{

    public LimelightHelpers.PoseEstimate megaTag2 = new PoseEstimate();
    public Supplier<Pigeon2> drivetrainGyro;

    public Pose2d testPose;
    public double testTimestamp;
    public RawFiducial[] fiducials;

    private int cameraRoll;


    public Vision(Supplier<Pigeon2> drivetrainGyro) {
        //Constructor.
        this.drivetrainGyro = drivetrainGyro;

        this.setLimelightRobotPosition();
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 0);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, this.getRobotHeading(), 0.0, 0.0, 0.0, 0.0, 0.0);
        this.testPose = new Pose2d(8.5, 4.0, new Rotation2d(0.0)); //Center of the field

        cameraRoll = 0;

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

    /**
     * Returns true is the pose estimate is not 'null.'
     * 'Valid' in this case means the limelight actually sent data and sees a valid target; I'm not actually checking if the data makes sense.
     * @param poseEstimate
     * @return
     */
    private boolean isMegaTagValid(LimelightHelpers.PoseEstimate poseEstimate) {
        return (poseEstimate != null) && LimelightHelpers.getTV(VisionConstants.kLimelightName);
    }

    /**
     * Returns true if the latest megaTag estimate sees at least the amount of tags passed into this method.
     * @param tagCount
     * @return
     */
    private boolean areTagsSeen(int tagCount) {
        return this.megaTag2.tagCount >= tagCount;
    }

    /**
     * Returns true if the rotational velocity of the robot is less than the value passed into this method.
     * @param rotationRate
     * @return
     */
    private boolean isRobotSlowEnough(double rotationRate) {
        return this.getRobotRotationRate() <= rotationRate;
    }

    private boolean isPoseValid() {
        //189.5 degrees per second is currently 75% of our maximum rotational speed.
        return this.isMegaTagValid(this.megaTag2) && this.areTagsSeen(1) && this.isRobotSlowEnough(189.5);
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
        return this.drivetrainGyro.get().getYaw().getValueAsDouble();
    }

    /**
     * Return the absolute angular velocity of the robot.
     * In degrees per second.
     * @return
     */
    private double getRobotRotationRate() {
        return Math.abs(this.drivetrainGyro.get().getAngularVelocityZDevice().getValueAsDouble());
    }

    private Pose2d getCurrentPose() {
        return this.megaTag2.pose;
    }

    private void takeSnapshot() {
        //In the future, this should generate a unique name for the snapshot.
        LimelightHelpers.takeSnapshot(VisionConstants.kLimelightName, "Snapshot-" + cameraRoll);
        this.cameraRoll++;
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    /**
     * A modified version of the consumer that the drivetrain uses for updating the robot odometry.
     * This is intended for debugging purposes only.  Notice that the method sets the default command of the subsystem.
     * In the future, the default command might need to be something else.
     * @param visionPose
     */
    public void registerTelemetry(Consumer<Pose2d> visionPose) {
        visionPose.accept(this.getPose().get());
        this.setDefaultCommand( this.runOnce(() -> {
            visionPose.accept(this.getPose().get());
        }).ignoringDisable(true));
    }

    public Trigger addLimelightPose = new Trigger(this::isPoseValid);
    public Trigger coralStationInView = new Trigger(this::coralStationInView);
    public Trigger bargeInView = new Trigger(this::bargeInView);
    public Trigger processorInView = new Trigger(this::processorInView);
    public Trigger reefInView = new Trigger(this::reefInView);

    public Command addMegaTag2(Supplier<CommandSwerveDrivetrain> drivetrain) {
        return run(
            () -> {
                testTimestamp = Utils.getCurrentTimeSeconds();
                drivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                drivetrain.get().addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
            }
        ).ignoringDisable(true);
    }

    public Command takePicture() {
        return runOnce(() -> {
            this.takeSnapshot();
        }
        ).withName("Take Picture").ignoringDisable(true);
    }

    private Supplier<Pose2d> getPose() {
        return this::getCurrentPose;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Robot Heading", this::getRobotHeading, null);
        builder.addDoubleProperty("Robot Rotation Velocity", this::getRobotRotationRate, null);
        builder.addBooleanProperty("Is Robot Slow Enough?", () -> {return this.isRobotSlowEnough(189.5);}, null);
        builder.addIntegerProperty("Camera Roll", () -> {return this.cameraRoll;}, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addDoubleProperty("megaTagPose X", () -> {return this.megaTag2.pose.getX();}, null);
        builder.addDoubleProperty("megaTagPose Y", () -> {return this.megaTag2.pose.getY();}, null);
        builder.addDoubleProperty("megaTagPose Rot", () -> {return this.megaTag2.pose.getRotation().getDegrees();}, null);

    }    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, this.getRobotHeading(), 0.0, 0.0, 0.0, 0.0, 0.0);
        //THIS LINE IS EXTREMELY IMPORTANT
        //Only update the megaTag if the estimate isn't null.
        if (this.isMegaTagValid(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName))) {
            megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);
        }

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

        this.testPose = new Pose2d(8.5, 4.0, Rotation2d.fromDegrees(this.getRobotHeading()));
    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
