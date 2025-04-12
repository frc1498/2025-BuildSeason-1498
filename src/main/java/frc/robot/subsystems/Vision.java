package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Kilo;
import static edu.wpi.first.units.Units.derive;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.VisionConstants;

public class Vision extends SubsystemBase{

    public LimelightHelpers.PoseEstimate megaTag2 = new PoseEstimate();
    public Supplier<Pigeon2> drivetrainState;
    public Consumer<Pose2d> visionPose;
    public CommandSwerveDrivetrain drivetrain;

    public Pose2d testPose;
    public double testTimestamp;
    public RawFiducial[] fiducials;

    private int cameraRoll;
    private double latestRobotHeading;
    private double latestRobotRotationRate;

    private Pose2d desiredReefLocation;
    private Pose2d desiredReefAlgaeLocation;
    private String reefLocation;

    private Pose2d reefA;
    private Pose2d reefB;
    private Pose2d reefC;
    private Pose2d reefD;
    private Pose2d reefE;
    private Pose2d reefF;
    private Pose2d reefG;
    private Pose2d reefH;
    private Pose2d reefI;
    private Pose2d reefJ;
    private Pose2d reefK;
    private Pose2d reefL;
    private Pose2d reefZ;

    private Pose2d reefAB;
    private Pose2d reefCD;
    private Pose2d reefEF;
    private Pose2d reefGH;
    private Pose2d reefIJ;
    private Pose2d reefKL;

    private Pose2d bargeLocation;

    public Vision(CommandSwerveDrivetrain drivetrain) {
        //Constructor.
        this.drivetrain = drivetrain;

        this.setLimelightRobotPosition();
        //In the constructor, set the IMU mode to 1, so the limelight IMU is seeded with the robot gyro heading.
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 1);
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, this.getRobotHeading(), 0.0, 0.0, 0.0, 0.0, 0.0);
        this.testPose = new Pose2d(8.5, 4.0, new Rotation2d(0.0)); //Center of the field

        cameraRoll = 0;
        latestRobotHeading = 0;
        latestRobotRotationRate = 0;

        reefA = AprilTagConstants.kBlueTag18Left;
        reefB = AprilTagConstants.kBlueTag18Right;
        reefC = AprilTagConstants.kBlueTag17Left;
        reefD = AprilTagConstants.kBlueTag17Right;
        reefE = AprilTagConstants.kBlueTag22Left;
        reefF = AprilTagConstants.kBlueTag22Right;
        reefG = AprilTagConstants.kBlueTag21Left;
        reefH = AprilTagConstants.kBlueTag21Right;
        reefI = AprilTagConstants.kBlueTag20Left;
        reefJ = AprilTagConstants.kBlueTag20Right;
        reefK = AprilTagConstants.kBlueTag19Left;
        reefL = AprilTagConstants.kBlueTag19Right;
        reefZ = AprilTagConstants.kBlueBarge;

        reefAB = AprilTagConstants.kBlueTag18;
        reefCD = AprilTagConstants.kBlueTag17;
        reefEF = AprilTagConstants.kBlueTag22;
        reefGH = AprilTagConstants.kBlueTag21;
        reefIJ = AprilTagConstants.kBlueTag20;
        reefKL = AprilTagConstants.kBlueTag19;

        bargeLocation = AprilTagConstants.kBlueBarge;

        desiredReefLocation = reefA;
        desiredReefAlgaeLocation = reefAB;
        reefLocation = "A";

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

    private void setLimelightToInternalIMU() {
        LimelightHelpers.SetIMUMode(VisionConstants.kLimelightName, 2);
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
    private boolean isRobotSlowEnough(double maximumRotationRate) {
        return this.latestRobotRotationRate <= maximumRotationRate;
    }

    private boolean isPoseValid() {
        //3.3 radians per second is currently 75% of our maximum rotational speed.
        return this.isMegaTagValid(this.megaTag2) && this.areTagsSeen(1) && this.isRobotSlowEnough(3.3);
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
        return this.drivetrain.getState().Pose.getRotation().getDegrees();
    }

    /**
     * Return the absolute angular velocity of the robot.
     * In radians per second.
     * @return
     */
    private double getRobotRotationRate() {
        return Math.abs(this.drivetrain.getState().Speeds.omegaRadiansPerSecond);
    }

    private Pose2d getCurrentPose() {
        return this.megaTag2.pose;
    }

    private void takeSnapshot() {
        //In the future, this should generate a unique name for the snapshot.
        LimelightHelpers.takeSnapshot(VisionConstants.kLimelightName, "Snapshot-" + cameraRoll);
        this.cameraRoll++;
    }

    private void setReefPositions(Alliance alliance) {
        switch (alliance) {
            case Blue:
                this.reefA = AprilTagConstants.kBlueTag18Left;
                this.reefB = AprilTagConstants.kBlueTag18Right;
                this.reefC = AprilTagConstants.kBlueTag17Left;
                this.reefD = AprilTagConstants.kBlueTag17Right;
                this.reefE = AprilTagConstants.kBlueTag22Left;
                this.reefF = AprilTagConstants.kBlueTag22Right;
                this.reefG = AprilTagConstants.kBlueTag21Left;
                this.reefH = AprilTagConstants.kBlueTag21Right;
                this.reefI = AprilTagConstants.kBlueTag20Left;
                this.reefJ = AprilTagConstants.kBlueTag20Right;
                this.reefK = AprilTagConstants.kBlueTag19Left;
                this.reefL = AprilTagConstants.kBlueTag19Right;

                this.reefAB = AprilTagConstants.kBlueTag18;
                this.reefCD = AprilTagConstants.kBlueTag17;
                this.reefEF = AprilTagConstants.kBlueTag22;
                this.reefGH = AprilTagConstants.kBlueTag21;
                this.reefIJ = AprilTagConstants.kBlueTag20;
                this.reefKL = AprilTagConstants.kBlueTag19;

                this.bargeLocation = AprilTagConstants.kBlueBarge;
            break;
            case Red:
                this.reefA = AprilTagConstants.kRedTag7Left;
                this.reefB = AprilTagConstants.kRedTag7Right;
                this.reefC = AprilTagConstants.kRedTag8Left;
                this.reefD = AprilTagConstants.kRedTag8Right;
                this.reefE = AprilTagConstants.kRedTag9Left;
                this.reefF = AprilTagConstants.kRedTag9Right;
                this.reefG = AprilTagConstants.kRedTag10Left;
                this.reefH = AprilTagConstants.kRedTag10Right;
                this.reefI = AprilTagConstants.kRedTag11Left;
                this.reefJ = AprilTagConstants.kRedTag11Right;
                this.reefK = AprilTagConstants.kRedTag6Left;
                this.reefL = AprilTagConstants.kRedTag6Right;

                this.reefAB = AprilTagConstants.kRedTag7;
                this.reefCD = AprilTagConstants.kRedTag8;
                this.reefEF = AprilTagConstants.kRedTag9;
                this.reefGH = AprilTagConstants.kRedTag10;
                this.reefIJ = AprilTagConstants.kRedTag11;
                this.reefKL = AprilTagConstants.kRedTag6;

                this.bargeLocation = AprilTagConstants.kRedBarge;
            break;
            default:
                this.reefA = AprilTagConstants.kBlueTag18Left;
                this.reefB = AprilTagConstants.kBlueTag18Right;
                this.reefC = AprilTagConstants.kBlueTag17Left;
                this.reefD = AprilTagConstants.kBlueTag17Right;
                this.reefE = AprilTagConstants.kBlueTag22Left;
                this.reefF = AprilTagConstants.kBlueTag22Right;
                this.reefG = AprilTagConstants.kBlueTag21Left;
                this.reefH = AprilTagConstants.kBlueTag21Right;
                this.reefI = AprilTagConstants.kBlueTag20Left;
                this.reefJ = AprilTagConstants.kBlueTag20Right;
                this.reefK = AprilTagConstants.kBlueTag19Left;
                this.reefL = AprilTagConstants.kBlueTag19Right;

                this.reefAB = AprilTagConstants.kBlueTag18;
                this.reefCD = AprilTagConstants.kBlueTag17;
                this.reefEF = AprilTagConstants.kBlueTag22;
                this.reefGH = AprilTagConstants.kBlueTag21;
                this.reefIJ = AprilTagConstants.kBlueTag20;
                this.reefKL = AprilTagConstants.kBlueTag19;

                this.bargeLocation = AprilTagConstants.kBlueBarge;
            break;
            
        }
    }

    private void setDesiredReefPosition(Supplier<String> reefLocation) {
        this.reefLocation = reefLocation.get();
        switch (this.reefLocation) {
            case "A":
                desiredReefLocation = this.reefA;
                desiredReefAlgaeLocation = this.reefAB;
            break;
            case "B":
                desiredReefLocation = this.reefB;
                desiredReefAlgaeLocation = this.reefAB;
            break;
            case "C":
                desiredReefLocation = this.reefC;
                desiredReefAlgaeLocation = this.reefCD;
            break;
            case "D":
                desiredReefLocation = this.reefD;
                desiredReefAlgaeLocation = this.reefCD;
            break;
            case "E":
                desiredReefLocation = this.reefE;
                desiredReefAlgaeLocation = this.reefEF;
            break;
            case "F":
                desiredReefLocation = this.reefF;
                desiredReefAlgaeLocation = this.reefEF;
            break;
            case "G":
                desiredReefLocation = this.reefG;
                desiredReefAlgaeLocation = this.reefGH;
            break;
            case "H":
                desiredReefLocation = this.reefH;
                desiredReefAlgaeLocation = this.reefGH;
            break;
            case "I":
                desiredReefLocation = this.reefI;
                desiredReefAlgaeLocation = this.reefIJ;
            break;
            case "J":
                desiredReefLocation = this.reefJ;
                desiredReefAlgaeLocation = this.reefIJ;
            break;
            case "K":
                desiredReefLocation = this.reefK;
                desiredReefAlgaeLocation = this.reefKL;
            break;
            case "L":
                desiredReefLocation = this.reefL;
                desiredReefAlgaeLocation = this.reefKL;
            break;
            default:
                desiredReefLocation = this.reefA;
                desiredReefAlgaeLocation = this.reefAB;
            break;   
        }
    }

    private Pose2d getDesiredReefPosition() {
        return this.desiredReefLocation;
    }

    private Pose2d getDesiredReefAlgaePosition() {
        return this.desiredReefAlgaeLocation;
    }

    private Pose2d getBargeLocation() {
        return this.bargeLocation;
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
        this.visionPose = visionPose;
        this.visionPose.accept(this.getPose().get());
        this.setDefaultCommand(this.updateLimelightTelemetry());
    }

    public Trigger addLimelightPose = new Trigger(this::isPoseValid);
    public Trigger coralStationInView = new Trigger(this::coralStationInView);
    public Trigger bargeInView = new Trigger(this::bargeInView);
    public Trigger processorInView = new Trigger(this::processorInView);
    public Trigger reefInView = new Trigger(this::reefInView);

    public Command updateLimelightTelemetry() {
        return runOnce(() -> {
            this.visionPose.accept(this.getPose().get());
        }).ignoringDisable(true)
        .withName("updateLimelightTelemetry");
    }

    public Command addMegaTag2(Supplier<CommandSwerveDrivetrain> drivetrain) {
        return this.updateLimelightTelemetry().andThen(run(
            () -> {
                testTimestamp = Utils.getCurrentTimeSeconds();
                drivetrain.get().setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                drivetrain.get().addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
            }
        ).withName("Adding Vision Measurement").ignoringDisable(true));
    }

    public Command takePicture() {
        return runOnce(() -> {
            this.takeSnapshot();
        }
        ).withName("Take Picture").ignoringDisable(true);
    }

    public Command switchToInternalIMU() {
        return runOnce(() -> {this.setLimelightToInternalIMU();}).withName("Setting Limelight to IMU Mode 2").ignoringDisable(true);
    }

    public Command setReefPosition(Supplier<String> reefLocation) {
        return runOnce(() -> {
            this.setDesiredReefPosition(reefLocation);
        }).withName("setReefPosition");
    }

    private Supplier<Pose2d> getPose() {
        return this::getCurrentPose;
    }

    public Supplier<Pose2d> getDesiredReefCoralPose() {
        return this::getDesiredReefPosition;
    }

    public Supplier<Pose2d> getDesiredReefAlgaePose() {
        return this::getDesiredReefAlgaePosition;
    }

    public Supplier<Pose2d> getBargePose() {
        return this::getBargeLocation;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //Sendable data for dashboard debugging will be added here.
        builder.addDoubleProperty("Robot Heading", () -> {return this.latestRobotHeading;}, null);
        builder.addDoubleProperty("Robot Rotation Velocity", () -> {return this.latestRobotRotationRate;}, null);
        builder.addBooleanProperty("Is Robot Slow Enough?", () -> {return this.isRobotSlowEnough(3.3);}, null);
        builder.addIntegerProperty("Camera Roll", () -> {return this.cameraRoll;}, null);
        builder.addStringProperty("Command", this::getCurrentCommandName, null);
        builder.addDoubleProperty("megaTagPose X", () -> {return this.megaTag2.pose.getX();}, null);
        builder.addDoubleProperty("megaTagPose Y", () -> {return this.megaTag2.pose.getY();}, null);
        builder.addDoubleProperty("megaTagPose Rot", () -> {return this.megaTag2.pose.getRotation().getDegrees();}, null);
        builder.addDoubleProperty("Reef Pose X", () -> {return this.reefA.getX();}, null);
        builder.addDoubleProperty("New Reef", () -> {return this.desiredReefLocation.getX();}, null);
        builder.addStringProperty("Desired Reef Location", () -> {return this.reefLocation;}, null);

    }    
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        latestRobotHeading = this.getRobotHeading();
        latestRobotRotationRate = this.getRobotRotationRate();
        LimelightHelpers.SetRobotOrientation(VisionConstants.kLimelightName, latestRobotHeading, 0.0, 0.0, 0.0, 0.0, 0.0);
        //THIS LINE IS EXTREMELY IMPORTANT
        //Only update the megaTag if the estimate isn't null.
        if (this.isMegaTagValid(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName))) {
            megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);
        }

        if(DriverStation.isDSAttached()) {
            this.setReefPositions(DriverStation.getAlliance().get());
        }

        //Get the raw fiducials.
        /*fiducials = LimelightHelpers.getRawFiducials(VisionConstants.kLimelightName);
        for (RawFiducial fiducial : fiducials) {
            int id = fiducial.id;                    // Tag ID
            double txnc = fiducial.txnc;             // X offset (no crosshair)
            double tync = fiducial.tync;             // Y offset (no crosshair)
            double ta = fiducial.ta;                 // Target area
            double distToCamera = fiducial.distToCamera;  // Distance to camera
            double distToRobot = fiducial.distToRobot;    // Distance to robot
            double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
        }*/


    }
  
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
