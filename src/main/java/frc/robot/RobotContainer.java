// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.File;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        //Auton commands need to be registered BEFORE the autons are loaded.
        //Autons are loaded after a trigger in configureBindings triggers, so this should be safe.
        registerAutonCommands();
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
        driver.rightTrigger(0.1).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
            andThen(move.intakeCoralFloorFrontToFront(endEffector.whatIsEndEffectorLocation())));

            //Rear To Front 
        driver.rightTrigger(0.1).and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_GROUND_PICKUP;}).
            andThen(move.intakeCoralFloorRearToFront(endEffector.whatIsEndEffectorLocation())));
/*
        //Driver - Coral Human Intake
            //Front To Rear
        driver.leftBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_HUMAN_PICKUP;}).
            andThen(move.intakeCoralHumanFrontToRear(endEffector.whatIsEndEffectorLocation())));
        
            //Rear To Rear
        driver.leftBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_HUMAN_PICKUP;}).
            andThen(move.intakeCoralHumanRearToRear(endEffector.whatIsEndEffectorLocation())));
 */


        //Driver - Spit Coral
            //Front To Front
        driver.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).and(wrist.isCanRange).
        onTrue(move.wristCoralRollerSpitFrontToFront(endEffector.whatIsEndEffectorLocation()));
            
        //Driver - Spit Coral
            //Rear To Front
        driver.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.and(wrist.isCanRange).negate()).
        onTrue(move.wristCoralRollerSpitRearToFront(endEffector.whatIsEndEffectorLocation()));

        //Driver - Climb
        driver.povDown().and(climber.isClimberReady).onTrue(climber.toClimberComplete());
          
        //Driver - Rezero Gyro
        driver.b().and(driver.x()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /*
        //Driver Clear Jams
        driver.a().and(climber.isClimberReady.negate()).onTrue(move.clearCoralIntake().andThen(move.clearJams()));
        */

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
            operator1.b().and(arm.isArmInFrontOfIntake).onTrue(
                Commands.parallel(move.coralL1FrontToFront(), endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;}))); 
            operator1.b().and(arm.isArmInFrontOfIntake.negate()).onTrue(
                    Commands.parallel(move.coralL1RearToFront(), endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;}))); 
  
                /*
        operator1.b().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.coralL1FrontToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;}))); 
            //Rear To Front
        operator1.b().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.coralL1RearToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;}))); 
        */


        
        //Operator - Score Coral L2
            //Front To Front
        operator1.y().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.coralL2FrontToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;})));
    
            //Rear To Front
        operator1.y().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.coralL2RearToFront().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;})));

        //Operator - Score Coral L3
            //Front To Rear
        operator1.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.coralL3FrontToRear().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;})));
    
            //Rear To Rear
        operator1.rightBumper().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.coralL3RearToRear().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;})));

        //Operator - Score Coral L4
            //Front To Rear
        operator1.start().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            move.coralL4FrontToRear().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;}))); //Score L4
    
            //Rear To Rear
        operator1.start().and(climber.isClimberReady.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            move.coralL4RearToRear().
            andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;}))); //Score L4

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
        operator1.back().and(arm.isArmInFrontOfIntake.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
            andThen(move.goToRemoveAlgaeL2FrontToFront(endEffector.whatIsEndEffectorLocation())));
            //Rear To Front
        operator1.back().and(arm.isArmInFrontOfIntake.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L2;}).
            andThen(move.goToRemoveAlgaeL2RearToFront(endEffector.whatIsEndEffectorLocation())));

        //Operator - Remove Algae L3
            //Front To Rear
        operator1.leftStick().and(arm.isArmInFrontOfIntake.negate()).and(arm.isArmInFrontOfIntake).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
            andThen(move.goToRemoveAlgaeL3FrontToRear(endEffector.whatIsEndEffectorLocation())));
            //Rear To Rear
        operator1.leftStick().and(arm.isArmInFrontOfIntake.negate()).and(arm.isArmInFrontOfIntake.negate()).onTrue(
            endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.ALGAE_L3;}).
            andThen(move.goToRemoveAlgaeL3RearToRear(endEffector.whatIsEndEffectorLocation())));

       //Operator - Climb
       operator1.x().and(operator1.a()).and(climber.isClimberReady.negate()).onTrue(climber.commandClimberUnLatch().
       andThen(move.clearClimb()).
       andThen(climber.climberTriggered()).
       andThen(climber.toClimberReady()).
       andThen(move.intakeToRaisedForClimb()).withName("Climber Ready"));

        //=============LED System==============================================
        //intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public void registerAutonCommands() {
/*
        NamedCommands.registerCommand("toCoralL1", move.coralL1().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L1;})));
        NamedCommands.registerCommand("toCoralL2", move.coralL2().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L2;})));
        NamedCommands.registerCommand("toCoralL3", move.coralL3().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L3;})));
        NamedCommands.registerCommand("toCoralL4", move.coralL4().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.CORAL_L4;})));
        NamedCommands.registerCommand("toCoralStow", move.coralStow().
        andThen(endEffector.setEndEffectorLocation(() -> {return endEffectorLocation.NONE;})));
        NamedCommands.registerCommand("spit", move.wristCoralRollerSpit(endEffector.whatIsEndEffectorLocation()).
        until(wrist.isPartInGripper.negate()).
        andThen(move.coralStow()));
  */
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

    public Trigger allianceCheck = new Trigger(() -> {return this.hasDeterminedAlliance;});
}
