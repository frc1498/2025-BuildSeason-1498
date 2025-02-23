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
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
import frc.robot.commands.Move;

public class RobotContainer {
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

    public final Move move = new Move(wrist, arm, intake, elevator);

    public LED leds = new LED();

    public String endEffectorMode="none";

    //Very important, the vision subsystem has to be created after the drivetrain.
    //The vision subsystem relies on creating a lambda that gets the drivetrain heading.
    public Vision vision = new Vision(() -> {return drivetrain.getPigeon2().getYaw().getValueAsDouble();});

    // Was throwing an error.  had to comment out the import as well
    //Future proofing CHRP functionality.
    //File chirpFolder = new File(Filesystem.getDeployDirectory() + "/chirp");
    File autonFolder = new File(Filesystem.getDeployDirectory() + "/pathplanner/autos");
    public Selector autoSelect = new Selector(autonFolder, ".auto", "Auto Selector");
    public ArrayList<Command> autonCommands = new ArrayList<Command>();

    public RobotContainer() {
        configureBindings();
        //loadAllAutonomous();
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

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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

        driver.leftBumper().onTrue(vision.addMegaTag2(() -> drivetrain));

        //====================Operator Commands========================
        //Button Correlation Table
        //===========
        //Operator 1 - NOTE!  These numbers may be off one, I can't remember if the array starts at 0 or 1.
        //A - DS 1 - Pickup: Algae Floor
        //B - DS 2 - 
        //X - DS 3 - Pickup: Algae L2
        //Y - DS 4 - Socre: Coral L1
        //leftBumper - DS 5 - Pikcup: Algae L3 
        //rightBumper - DS 6  - Score: Coral L2
        // Select - DS 7 - Pickup: Coral Human
        // Start - DS 8  - Score: Coral L3
        // Left Stick Press - DS 9 - Pickup: Coral Floor
        // Right Stick Press - DS 10 - Score: Coral L4 / Barge
        //===========
        //Operator 2
        //A - DS 1 - 
        //B - DS 2 - Stow
        //X - DS 3 - 
        //Y - DS 4 - 
        //leftBumper - DS 5  - 
        //rightBumper - DS 6  - 
        // Select - DS 7 - Climber - Load
        // Start - DS 8  - Descore: Algae L2
        // Left Stick Press - DS 9 - Climber - Load
        // Right Stick Press - DS 10 - Descore: Algae L3

        /* PseudoCode - Intake Suck in coral ground mode
         * When Right trigger and not algae mode,
         * then move intake to floor and wait until it is there
         * then move endeffector into coral pickup position
         * then turn on intake rollers and end effector wrist rollers until a part breaks the forward gripper
         * then stop the rollers and start the position code for the coral 
         * and then move end effector to coral stow
         * then raise the intake
         */

        //==========================Intake Coral from Ground=============================
        driver.rightTrigger(0.1).and(climber.isClimberReady.negate()).onTrue(move.intakeCoralFloor());
     
        driver.leftBumper().onTrue(move.wristCoralRollerSpit()).onFalse(move.wristCoralRollerStop());
     
        //.and(move.wrist.isRangeOk)

        //==========================Intake Coral from Human================================    
        //driver.rightBumper().and(climber.isClimberReady.negate()).and(endEffector.isModeAlgae.negate()).onTrue(    
        //    endEffectorCommand.toCoralHumanPickup().until(endEffectorCommand.isEndEffectorAtCoralHumanPickup)
        //    .andThen(endEffectorCommand.wrist.suck()).until(endEffectorCommand.wrist.isPartForwardGripper)
        //    .andThen(endEffectorCommand.wrist.positionCoralInGripper()).andThen(endEffectorCommand.toCoralStow()));

        //==============================Algae Intake=====================================    
        /*  Algae suck 
            Move End effector to algae pickup location while raising the intake
            then
            
            driver.rightTrigger(0.1).and(endEffector.isModeAlgae).onTrue();  //Intake Suck in algae mode*/

        //=================================Spit Coral====================================    
        //driver.y().and(climber.isClimberReady.negate()).and(endEffector.isModeAlgae.negate()).onTrue(
        //intake.intakeFloor().andThen(Commands.parallel(intake.rollerSpit(), endEffectorCommand.wrist.suck()))).onFalse(intake.intakeRaised());

        //=============================== Spit Algae=====================================
        
        //================================Score Coral=====================================
        //driver.leftBumper().and(climber.isClimberReady.negate()).and(endEffector.isModeAlgae.negate()).onTrue(
        //endEffectorCommand.wrist.spit().until(endEffectorCommand.wrist.isPartInGripper)
        //.andThen(endEffectorCommand.toCoralStow()));      
        
        /* =================================Score Algae====================================
        driver.leftBumper().onTrue();
        */

        //==================================Climb==========================================
        driver.povDown().and(climber.isClimberReady).onTrue(climber.toClimberComplete());
        
        //Auton Selection
        driver.povLeft().onTrue(autoSelect.decrement());
        driver.povRight().onTrue(autoSelect.increment());
        
        //=====================================================================
        //=============================Operator 1==============================
        //=====================================================================
        //operator1.a().onTrue();
        //operator1.x().onTrue();
        //operator1.back().onTrue();
        //operator1.leftBumper().onTrue();

        //============================Operator to Coral L1=======================
        operator1.b().and(climber.isClimberReady.negate()).onTrue(move.coralL1()); //Score L1, Processor
        
        //============================Operator to Processor=======================
        //operator1.b().and(endEffector.isModeAlgae.negate())onTrue(endEffectorCommand.moveEndEffector("L1orProcessor")); //Score L1, Processor

        //============================Operator ot Coral L3====================================
        operator1.y().and(climber.isClimberReady.negate()).onTrue(move.coralL2()); //Score L3

        //============================Operator ot Algae L3====================================
        operator1.rightBumper().and(climber.isClimberReady.negate()).onTrue(move.coralL3()); //Score L3

        //============================Operator to Coral L4=============================
        operator1.start().and(climber.isClimberReady.negate()).onTrue(move.coralL4()); //Score L4, Barge

        //============================Operator to Barge=============================
        //operator1.start().onTrue(endEffectorCommand.moveEndEffector("L4orBarge")); //Score L4, Barge

        //============================Operator to Stow==============================
        operator1.rightStick().onTrue(move.coralStow());

        //===============================Select Mode=====================================
        //operator1.leftStick().onTrue(endEffector.setEndEffectorMode("Coral"));  //Coral Mode
        //

        //=====================================================================
        //=============================Operator 2==============================
        //=====================================================================

        //==============Trigger Climber========================================
        //operator1.a().and(climber.isClimberReady.negate()).and(operator1.x()).onTrue(climber.climberTriggered().andThen(endEffectorCommand.toCoralL1()).andThen(climber.toClimberReady()));


        //operator2.b().onTrue();
        //operator2.x().onTrue();
        //operator2.back().onTrue();
        //operator2.start().onTrue());  //Descore Algae L2
        //operator2.rightBumper().onTrue();
        //operator2.leftBumper().onTrue();
        //operator2.leftStick().onTrue();
        //operator2.rightStick().onTrue();  //Descore Algae L3

        //=============LED System==============================================
        //intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Blue Simple");
    }

    public void loadAllAutonomous(Supplier<ArrayList<String>> autonList) {
        ArrayList<Command> commandList= new ArrayList<Command>();
        for (var i : autonList.get()) {
            commandList.add(new PathPlannerAuto(i));
        }
    }
}
