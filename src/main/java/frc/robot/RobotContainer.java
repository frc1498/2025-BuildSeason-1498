// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
import frc.robot.constants.EndEffectorConstants.endEffectorLocation;
import frc.robot.commands.Move;

public class RobotContainer {
    endEffectorLocation endEffectorlocation;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Instantiate 
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator1 = new CommandXboxController(1);
    //private final CommandXboxController operator2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Temporarily commented out to test placing the subsystems in an end effector system
    public final ElevatorConfig elevatorConfig = new ElevatorConfig();
    public Elevator elevator = new Elevator(elevatorConfig);

    public final WristConfig wristConfig = new WristConfig();
    public Wrist wrist = new Wrist(wristConfig);
    
    public final ArmConfig armConfig = new ArmConfig();
    public Arm arm = new Arm(armConfig);
    */
    //=======================================================================
    //=======================Assign Subsystem Names==========================
    //=======================================================================
    public final CoralIntakeConfig intakeConfig = new CoralIntakeConfig();
    public CoralIntake intake = new CoralIntake(intakeConfig);

    public ArmConfig armConfig = new ArmConfig();
    public Arm arm = new Arm(armConfig);
    
    public WristConfig wristConfig = new WristConfig();
    public Wrist wrist = new Wrist(wristConfig);
    
    public ElevatorConfig elevatorConfig = new ElevatorConfig();
    public Elevator elevator = new Elevator(elevatorConfig);

    public final ClimberConfig climberConfig = new ClimberConfig();
    public final Climber climber = new Climber(climberConfig);

    public final EndEffector endEffector = new EndEffector();

    public final Move move = new Move(wrist, arm, intake, elevator);

    public LED leds = new LED();

    //Very important, the vision subsystem has to be created after the drivetrain.
    //The vision subsystem relies on creating a lambda that gets the drivetrain heading.
    public Vision vision = new Vision(() -> {return drivetrain.getPigeon2().getYaw().getValueAsDouble();});

    //Future proofing CHRP functionality.
    //File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
    File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    public Selector autoSelect = new Selector(autonFolder, ".auto", "Auto Selector");
    //All of these are needed to display and load the correct list of autonomous options.
    public ArrayList<Command> autonCommands = new ArrayList<Command>();
    public Command selectedAuton;
    public Alliance allianceColor = Alliance.Blue;
    public boolean hasDeterminedAlliance;

    public RobotContainer() {
        
        //autonCommands = loadAllAutonomous(autoSelect.currentList());
        registerAutonTriggers();
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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //driver.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        
     
        // reset the field-centric heading on left bumper press
        //driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Makes testing difficult, but sim makes it less annoying.
        //Does three things:
        //1. Filters the list of autons by alliance color.
        //2. Loads all of the routines into auton commands that PathPlanner can run.
        //3. Select the first auton in the list by default.
        this.allianceCheck.onTrue(autoSelect.filterList(() -> {return allianceColor.toString();})
            .andThen(() -> {autonCommands = this.loadAllAutonomous(autoSelect.currentList());}).ignoringDisable(true)
            .andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));

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

        //driver.leftBumper().onTrue(vision.addMegaTag2(() -> drivetrain));
        
        //Auton Selection
        driver.povLeft().onTrue(autoSelect.decrement().andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));
        driver.povRight().onTrue(autoSelect.increment().andThen(() -> {selectedAuton = autonCommands.get(autoSelect.currentIndex().get());}).ignoringDisable(true));

        //=====================================================================
        //==============================Driver=================================
        //=====================================================================
        driver.rightTrigger(0.1).and(climber.isClimberReady.negate()).onTrue(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
            andThen(move.intakeCoralFloor(endEffector.whatIsEndEffectorLocation())));  //Intake Coral from Ground
 
        driver.leftBumper().and(climber.isClimberReady.negate()).onTrue(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_HUMAN_PICKUP;}).
            andThen(move.intakeCoralHuman(endEffector.whatIsEndEffectorLocation())));  //Intake Coral from Human
 
        driver.rightBumper().onTrue(move.wristCoralRollerSpit(endEffector.whatIsEndEffectorLocation()).
            until(wrist.isPartInGripper.negate()).
            andThen(move.coralStow()));  //Spit Coral
 
        driver.povDown().and(climber.isClimberReady).onTrue(climber.toClimberComplete().
        andThen(climber.commandClimberLatch())); //Climb
          
        //=====================================================================
        //=============================Operator 1==============================
        //=====================================================================
        operator1.b().and(climber.isClimberReady.negate()).onTrue(move.coralL1().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;}))); //Score L1

        operator1.y().and(climber.isClimberReady.negate()).onTrue(move.coralL2().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;}))); //Score L3

        operator1.rightBumper().and(climber.isClimberReady.negate()).onTrue(move.coralL3().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;}))); //Score L3

        operator1.start().and(climber.isClimberReady.negate()).onTrue(move.coralL4().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;}))); //Score L4

        operator1.rightStick().onTrue(move.coralStow().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.NONE;})));  //Coral Stow

        operator1.x().and(operator1.a()).onTrue(climber.commandClimberUnLatch().
        andThen(move.clearClimb()).
        andThen(climber.climberTriggered()).
        andThen(climber.toClimberReady()).
        andThen(move.intakeToRaisedForClimb()).withName("Climber Ready"));

        //===============================Select Mode=====================================
        //operator1.leftStick().onTrue(endEffector.setEndEffectorMode("Coral"));  //Coral Mode
        //

        //=============LED System==============================================
        //intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        return selectedAuton;
    }

    public ArrayList<Command> loadAllAutonomous(Supplier<ArrayList<String>> autonList) {
        ArrayList<Command> commandList= new ArrayList<Command>();
        for (var i : autonList.get()) {
            commandList.add(new PathPlannerAuto(i));
        }

        return commandList;
    }

    public void registerAutonTriggers() {
        EventTrigger atCoralL4Auton = new EventTrigger("CoralL4");
    }

    public Trigger allianceCheck = new Trigger(() -> {return this.hasDeterminedAlliance;});
}
