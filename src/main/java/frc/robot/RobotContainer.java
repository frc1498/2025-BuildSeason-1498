// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.pilotLib.utility.Selector;

import frc.robot.config.CoralIntakeConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.config.ArmConfig;
import frc.robot.config.ClimberConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.EndEffector;
import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.EndEffectorConstants.endEffectorLocation;
import frc.robot.commands.Move;

public class RobotContainer {
    endEffectorLocation endEffectorlocation;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double precisionDampener = 1.0; //This makes it sound very cool.

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.005).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric slideSideways = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    //  <- deadband

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Instantiate 
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator1 = new CommandXboxController(1);
    private final CommandXboxController operator2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //=======================================================================
    //=======================Assign Subsystem Names==========================
    //=======================================================================
    public final EndEffector endEffector = new EndEffector();
    
    public final CoralIntakeConfig intakeConfig = new CoralIntakeConfig();
    public CoralIntake intake = new CoralIntake(intakeConfig);

    public ArmConfig armConfig = new ArmConfig();
    public Arm arm = new Arm(armConfig);
    
    public WristConfig wristConfig = new WristConfig();
    public Wrist wrist = new Wrist(wristConfig, endEffector.whatIsEndEffectorLocation());
    
    public ElevatorConfig elevatorConfig = new ElevatorConfig();
    public Elevator elevator = new Elevator(elevatorConfig);

    public final ClimberConfig climberConfig = new ClimberConfig();
    public final Climber climber = new Climber(climberConfig);

    public final Move move = new Move(wrist, arm, intake, elevator);

    public LED leds = new LED();

    //Very important, the vision subsystem has to be created after the drivetrain.
    //The vision subsystem relies on creating a lambda that gets the drivetrain gyro.
    public Vision vision = new Vision(drivetrain);

    //Makin' music.
    public Orchestra music = new Orchestra();

    //Future proofing CHRP functionality.
    File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
    File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    public Selector autoSelect = new Selector(autonFolder, ".auto", "Auto Selector");
    public Selector chirpSelect = new Selector(chirpFolder, ".chrp", "Chirp Selector");
    //All of these are needed to display and load the correct list of autonomous options.
    public ArrayList<Command> autonCommands = new ArrayList<Command>();
    public Command selectedAuton;
    public Alliance allianceColor = Alliance.Blue;
    public boolean hasDeterminedAlliance;

    public RobotContainer() {
        //Auton commands need to be registered BEFORE the autons are loaded.
        //Autons are loaded after a trigger in configureBindings triggers, so this should be safe.
        registerAutonCommands();
        registerOrchestra();
        configureBindings();
    }

    private void configureBindings() {

        //=========================================================================
        //=============================Driver Drivetrain===========================
        //=========================================================================
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-(Math.pow(driver.getLeftY() * precisionDampener,3)) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-(Math.pow(driver.getLeftX() * precisionDampener,3)) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * precisionDampener) // Drive counterclockwise with negative X (left)
            )
        );




        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        //Makes testing difficult, but sim makes it less annoying.
        //Does three things:
        //1. Filters the list of autons by alliance color.
        //2. Loads all of the routines into auton commands that PathPlanner can run.
        //3. Select the first auton in the list by default.
        this.allianceCheck.onTrue(autoSelect.filterList(() -> {return allianceColor.toString();})
            .andThen(() -> {autonCommands = this.loadAllAutonomous(autoSelect.currentList());}).ignoringDisable(true)
            .andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));        
    
        //Auton Selection
        driver.povLeft().onTrue(autoSelect.decrement().andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));
        driver.povRight().onTrue(autoSelect.increment().andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));            

        //Music Selection
        driver.povUp().onTrue(chirpSelect.decrement().andThen(() -> {music.loadMusic(Filesystem.getDeployDirectory() + "/chirp/" + chirpSelect.getCurrentSelectionName() + ".chrp");}).ignoringDisable(true));
        driver.povDown().onTrue(chirpSelect.increment().andThen(() -> {music.loadMusic(Filesystem.getDeployDirectory() + "/chirp/" + chirpSelect.getCurrentSelectionName() + ".chrp");}).ignoringDisable(true));
        
        //===================================================================================
        //=============================Driver Commands=======================================
        //===================================================================================
        
        /* SIMULATION PRACTICE COMMANDS
        //Elevator Simulation Practice Commands
        driver.povDown().onTrue(elevator.elevatorCoralL1());
        driver.povLeft().onTrue(elevator.elevatorCoralL2());
        driver.povUp().onTrue(elevator.elevatorCoralL3());
        driver.povRight().onTrue(elevator.elevatorHome());
        driver.rightTrigger(0.1).whileTrue(elevator.elevatorPosition(() -> {return driver.getRightTriggerAxis() * 2;}));
        driver.leftBumper().whileTrue(intake.rollerSuck());
        
        driver.y().onTrue(arm.armCoralL1()).onFalse(arm.armCoralL2());
        driver.povDown().onTrue(endEffector.toCoralL1());
        driver.povLeft().onTrue(endEffector.toCoralL2());
        driver.povUp().onTrue(endEffector.toCoralL3());
        driver.povRight().onTrue(endEffector.toCoralStow());
        driver.rightBumper().onTrue(endEffector.toCoralSuck());
        */


        //=====================================================================
        //==============================Driver=================================
        //=====================================================================    
        //Drive - Coral Floor Intake
        //Front To Front    
        driver.rightTrigger(0.1).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.intakeCoralFloorFrontToFront(endEffector.whatIsEndEffectorLocation())).
            andThen(move.intakeCoralFloorFrontToFrontReturn()));

        //Driver - Coral Human Intake
            //Front To Rear
        driver.povUp().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_HUMAN_PICKUP;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.intakeCoralHumanFrontToRear()));

        //Driver - Spit Coral
        //Slide left 
        //driver.pov(90).whileTrue(drivetrain.applyRequest(() -> slideSideways.withVelocityX(0.015).withVelocityY(-.3)));

        driver.b().whileTrue(drivetrain.applyRequest(() -> slideSideways.withVelocityX(0.015).withVelocityY(-.3)));

        //Front to Front
        /*
        driver.pov(90).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitFrontToFront(endEffector.whatIsEndEffectorLocation()));
        */

        //Rear to Front
        /*
        driver.pov(90).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));
        */

        //Slide right
        driver.x().whileTrue(drivetrain.applyRequest(() -> slideSideways.withVelocityX(.015).withVelocityY(.3)));

        //Front to Front
        /*
        driver.pov(270).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitFrontToFront(endEffector.whatIsEndEffectorLocation()));
        */

        //Rear to Front
        /*
        driver.pov(270).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));
        */
              
        //Driver - Spit Coral
        //Front To Front
        driver.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitFrontToFront(endEffector.whatIsEndEffectorLocation()));

        
        //Driver - Slow down by 50% while holding the spit button
        driver.rightBumper().onTrue(this.setDampener())
        .onFalse(this.resetDampener());
        

        //Driver - Spit Coral
        //Rear To Front
        driver.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));

        //Driver - Spit Coral No Range Sensor
        //Front to Front
        driver.leftBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).
        onTrue(move.wristCoralRollerSpitFrontToFront(endEffector.whatIsEndEffectorLocation()));
        
        
        //Driver - Spit Coral No Range Sensor
        //Rear to Front
        driver.leftBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).
        onTrue(move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));
        
        //Drive to the 'scoring position' - either a post on the reef, or inbetween two posts for algae.
        //I'm sure there's a better way to do this check - probably in Vision.java - but this makes the most sense immediately at 9:30 PM.
        driver.leftTrigger(.1).onTrue(Commands.select(
            Map.ofEntries(
                Map.entry("Coral", drivetrain.pathPlannerToPose(vision.getDesiredReefCoralPose())), 
                Map.entry("Algae", drivetrain.pathPlannerToPose(vision.getDesiredReefAlgaePose()))),
            endEffector.getEndEffectorMode()));

        //Driver - Climb
        driver.povDown().and(climber.isClimberReady).onTrue(climber.toClimberComplete());
          
        //Driver - Rezero Gyro
        driver.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        //Driver - Score Algae
        driver.start().whileTrue(move.spitAlgae()).onFalse(move.coralStowRearToFront());

        //Driver - Algae Stow - Front to Rear
        driver.a().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            Commands.parallel(move.AlgaeStowFrontToRear(),drivetrain.pathPlannerToPose(vision.getBargePose())).
            andThen(move.AlgaeScoreRearToRear()).
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L4;})));

        //Driver - Algae Stow - Rear to Rear
        driver.a().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            Commands.parallel(move.AlgaeStowFrontToRear(),drivetrain.pathPlannerToPose(vision.getBargePose())).
            andThen(move.AlgaeScoreRearToRear()).
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L4;})));

        /*
        //Algae Stow Front to Rear
        driver.a().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.AlgaeStowFrontToRear());

        //Driver - Algae Stow - Rear to Rear
        driver.a().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.AlgaeStowFrontToRear());
        */


        //Drive - Algae Score - Rear to Rear
        driver.y().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.AlgaeScoreRearToRear().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L4;})));

        //=====================================================================
        //=============================Operator 1==============================
        //=====================================================================
        // To Do:  If we use finite states
        //  Check trevor's arm state.  How many positions do we need?  Taken from a trigger:
        //      Forward of intake
        //      Behind intake
        //  Each button press will need these 2 states
        //
        //  How many Commands will we need in move for general moves?
        //      Front To Front
        //      Front To Rear
        //      Rear To Rear
        //      Rear To Front


        //Operator - Score Coral L1
            //Front To Front
        operator1.b().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL1FrontToFront())); 
            //Rear To Front
        operator1.b().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL1RearToFront())); 

        //Operator - Score Coral L2
            //Front To Front
        operator1.y().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL2FrontToFront()));
    
            //Rear To Front
        operator1.y().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;}).   
            andThen(endEffector.setEndEffectorMode("Coral")). 
            andThen(move.coralL2RearToFront()));

        //Operator - Score Coral L3
            //Front To Rear
        operator1.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL3FrontToRear()));
    
            //Rear To Rear
        operator1.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL3RearToRear()));

        //Operator - Score Coral L4
            //Front To Rear
        operator1.start().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL4FrontToRear())); //Score L4
    
            //Rear To Rear
        operator1.start().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;}).
            andThen(endEffector.setEndEffectorMode("Coral")).
            andThen(move.coralL4RearToRear())); //Score L4

        //Operator - Coral Stow
            //Front To Front
        operator1.rightStick().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.coralStowFrontToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.NONE;})));

            //Rear To Front
        operator1.rightStick().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.coralStowRearToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.NONE;})));

        //Operator - Remove Algae L2
            //Front To Front
        operator1.back().and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
            andThen(endEffector.setEndEffectorMode("Algae")).
            andThen(move.goToRemoveAlgaeL2FrontToFront(endEffector.whatIsEndEffectorLocation())));
            //Rear To Front
        operator1.back().and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
            andThen(endEffector.setEndEffectorMode("Algae")).
            andThen(move.goToRemoveAlgaeL2RearToFront(endEffector.whatIsEndEffectorLocation())));

        //Operator - Remove Algae L3
            //Front To Front
        operator1.leftStick().and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
            andThen(endEffector.setEndEffectorMode("Algae")).
            andThen(move.goToRemoveAlgaeL3FrontToFront(endEffector.whatIsEndEffectorLocation())));
            //Rear To Front
        operator1.leftStick().and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
            andThen(endEffector.setEndEffectorMode("Algae")).
            andThen(move.goToRemoveAlgaeL3RearToFront(endEffector.whatIsEndEffectorLocation())));

       //Operator - Climb
       operator2.x().and(operator2.start()).and(climber.isClimberReady.negate()).onTrue(move.clearClimb().
       andThen(climber.climberTriggered()).
       andThen(climber.toClimberReady()).
       andThen(move.intakeToRaisedForClimb()).withName("Climber Ready"));

        //=============LED System==============================================
        //intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        driver.leftBumper().and(driver.rightBumper()).onTrue(vision.takePicture());
        vision.addLimelightPose.whileTrue(vision.addMegaTag2(() -> {return drivetrain;}));
        //autonomousStarted.onTrue(vision.switchToInternalIMU());

        //Music
        driver.rightBumper().and(RobotState::isDisabled)
        .onTrue(new InstantCommand(() -> {music.play();}).ignoringDisable(true))
        .onFalse(new InstantCommand(() -> {music.stop();}).ignoringDisable(true));
        
        drivetrain.registerTelemetry(logger::telemeterize);
        vision.registerTelemetry(logger::visionTelemeterize);

        //driver.start().onTrue(drivetrain.pathPlannerToPose(vision.getDesiredReefPose()));
        driver.back().onTrue(drivetrain.abortPathPlanner());

        driver.axisMagnitudeGreaterThan(0, 0.1)
        .or(driver.axisMagnitudeGreaterThan(1, 0.1))
        .or(driver.axisMagnitudeGreaterThan(4, 0.1))
        .or(driver.axisMagnitudeGreaterThan(5, 0.1))
        .onTrue(drivetrain.abortPathPlanner());
        
        //Auto-Align - setting of the reef positions.
        operator2.rightBumper().onTrue(vision.setReefPosition(() -> {return "A";}));    //1L
        operator2.leftBumper().onTrue(vision.setReefPosition(() -> {return "B";}));    //1R
        operator2.y().onTrue(vision.setReefPosition(() -> {return "C";}));  //6L
        operator2.x().onTrue(vision.setReefPosition(() -> {return "D";}));  //6R
        operator2.b().onTrue(vision.setReefPosition(() -> {return "E";}));  //5L
        operator2.a().onTrue(vision.setReefPosition(() -> {return "F";}));  //5R
        operator1.a().onTrue(vision.setReefPosition(() -> {return "G";}));  //4L
        operator1.x().onTrue(vision.setReefPosition(() -> {return "H";}));  //4R
        operator2.rightStick().onTrue(vision.setReefPosition(() -> {return "I";}));  //3L
        operator2.leftStick().onTrue(vision.setReefPosition(() -> {return "J";})); //3R
        operator2.start().onTrue(vision.setReefPosition(() -> {return "K";}));  //2L
        operator2.back().onTrue(vision.setReefPosition(() -> {return "L";}));   //2R

    }

    public void registerAutonCommands() {
        NamedCommands.registerCommand("toCoralL4FrontToRear", move.coralL4FrontToRear().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;})));
        NamedCommands.registerCommand("spitRearToFront", move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));
        NamedCommands.registerCommand("toCoralStow", move.coralStowRearToFront().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.NONE;})));        
        NamedCommands.registerCommand("intake", endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
        andThen(move.intakeCoralFloorFrontToFront(endEffector.whatIsEndEffectorLocation())));
        NamedCommands.registerCommand("intakeReturn", move.intakeCoralFloorFrontToFrontReturn());
        NamedCommands.registerCommand("humanIntake", endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
        andThen(move.intakeCoralHumanRearToRearAuto()));
        NamedCommands.registerCommand("spit", move.spitAuto());
        NamedCommands.registerCommand("toCoralL4RearToRear", move.coralL4RearToRear().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;})));
        NamedCommands.registerCommand("AlgaeStowFrontToRear", move.AlgaeStowFrontToRear());
        NamedCommands.registerCommand("AlgaeScore", move.AlgaeScoreFrontToRear().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L4;})));
        NamedCommands.registerCommand("AlgaeSpit", move.spitAlgae());
        NamedCommands.registerCommand("AlgaeStowRearToFront", move.coralStowRearToFront());
        NamedCommands.registerCommand("AlgaeL2FrontToFront", endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
        andThen(endEffector.setEndEffectorMode("Algae")).
        andThen(move.goToRemoveAlgaeL2FrontToFront(endEffector.whatIsEndEffectorLocation())));
        NamedCommands.registerCommand("AlgaeL2RearToFront", endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
        andThen(endEffector.setEndEffectorMode("Algae")).
        andThen(move.goToRemoveAlgaeL2RearToFront(endEffector.whatIsEndEffectorLocation())));
        NamedCommands.registerCommand("AlgaeL3FrontToFront", 
        endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
        andThen(endEffector.setEndEffectorMode("Algae")).
        andThen(move.goToRemoveAlgaeL3FrontToFront(endEffector.whatIsEndEffectorLocation())));
        NamedCommands.registerCommand("AlgaeL3RearToFront", endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
        andThen(endEffector.setEndEffectorMode("Algae")).
        andThen(move.goToRemoveAlgaeL3RearToFront(endEffector.whatIsEndEffectorLocation())));
        /*

        NamedCommands.registerCommand("toCoralL2", move.coralL2().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;})));
        NamedCommands.registerCommand("toCoralL3", move.coralL3().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;})));
        


  */
    }

    public void registerOrchestra() {
        intake.addToOrchestra(music);
        arm.addToOrchestra(music);
        wrist.addToOrchestra(music);
        elevator.addToOrchestra(music);
        climber.addToOrchestra(music);
        drivetrain.addToOrchestra(music);
    }

    public Command getAutonomousCommand() {
        return selectedAuton;
    }

    public Command setDampener() {
        return Commands.runOnce( () -> {
            this.precisionDampener = 0.55;
        });
    }

    public Command resetDampener() {
        return Commands.runOnce( () -> {
            this.precisionDampener = 1.0;
        });
    }

    public ArrayList<Command> loadAllAutonomous(Supplier<ArrayList<String>> autonList) {
        ArrayList<Command> commandList= new ArrayList<Command>();
        for (var i : autonList.get()) {
            commandList.add(new PathPlannerAuto(i));
        }

        return commandList;
    }

    public Trigger allianceCheck = new Trigger(() -> {return this.hasDeterminedAlliance;});
    public Trigger autonomousStarted = new Trigger(RobotState::isAutonomous);   
}
