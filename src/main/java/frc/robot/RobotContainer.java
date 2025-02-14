// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.config.ArmConfig;
import frc.robot.config.CoralIntakeConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.constants.EndEffectorConstants;
import frc.robot.config.ClimberConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.EndEffector;

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
    private final CommandXboxController operator2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Temporarily commented out to test placing the subsystems in an end effector system
    public final ElevatorConfig elevatorConfig = new ElevatorConfig();
    public Elevator elevator = new Elevator(elevatorConfig);

    public final WristConfig wristConfig = new WristConfig();
    public Wrist wrist = new Wrist(wristConfig);
    
    public final ArmConfig armConfig = new ArmConfig();
    public Arm arm = new Arm(armConfig);
    */

    public final CoralIntakeConfig intakeConfig = new CoralIntakeConfig();
    public CoralIntake intake = new CoralIntake(intakeConfig);

    public EndEffectorCommand endEffectorCommand = new EndEffectorCommand();
    public final EndEffector endEffector = new EndEffector();

    public final ClimberConfig climberConfig = new ClimberConfig();
    public final Climber climber = new Climber(climberConfig);

    public LED leds = new LED();

    public String endEffectorMode="none";

    //Very important, the vision subsystem has to be created after the drivetrain.
    //The vision subsystem relies on creating a lambda that gets the drivetrain heading.
    public Vision vision = new Vision(() -> {return drivetrain.getPigeon2().getYaw().getValueAsDouble();});

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
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
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /* SIMULATION PRACTICE COMMANDS
        //Elevator Simulation Practice Commands
        driver.povDown().onTrue(elevator.elevatorCoralL1());
        driver.povLeft().onTrue(elevator.elevatorCoralL2());
        driver.povUp().onTrue(elevator.elevatorCoralL3());
        driver.povRight().onTrue(elevator.elevatorHome());
        driver.rightTrigger(0.1).whileTrue(elevator.elevatorPosition(() -> {return driver.getRightTriggerAxis() * 2;}));
        driver.leftBumper().whileTrue(intake.rollerSuck());
        driver.leftTrigger(0.1).onTrue(wrist.wristCoralL1()).onFalse(wrist.wristCoralL2());
        driver.y().onTrue(arm.armCoralL1()).onFalse(arm.armCoralL2());
        driver.povDown().onTrue(endEffector.toCoralL1());
        driver.povLeft().onTrue(endEffector.toCoralL2());
        driver.povUp().onTrue(endEffector.toCoralL3());
        driver.povRight().onTrue(endEffector.toCoralStow());
        driver.rightBumper().onTrue(endEffector.toCoralSuck());
        */


        
        
        //=====================================================================
        //=============================Operator 1==============================
        //=====================================================================
        //operator1.a().onTrue();
        operator1.b().onTrue(new callEndEffector("L1orProcessor")); //Score L1, Processor
        operator1.y().onTrue(new callEndEffector("L2")); //Score L2
        //operator1.x().onTrue();
        //operator1.back().onTrue();
        operator1.start().onTrue(new callEndEffector("L4orBarge")); //Score L4, Barge
        operator1.rightBumper().onTrue(new callEndEffector("L3")); //Score L3
        //operator1.leftBumper().onTrue();
        operator1.leftStick().onTrue(endEffector.setEndEffectorMode("Coral"));  //Coral Mode
        operator1.rightStick().onTrue(endEffector.setEndEffectorMode("Algae"));  //Algae Mode

        //=====================================================================
        //=============================Operator 2==============================
        //=====================================================================
        operator2.a().onTrue();   //Climber Load
        //operator2.b().onTrue();
        operator2.y().onTrue();  //Climber Load
        //operator2.x().onTrue();
        //operator2.back().onTrue();
        operator2.start().onTrue(new callEndEffector("DescoreL2"));  //Descore Algae L2
        //operator2.rightBumper().onTrue();
        //operator2.leftBumper().onTrue();
        //operator2.leftStick().onTrue();
        operator2.rightStick().onTrue(endEffector.callEndEffector("DescoreL3"));  //Descore Algae L3

        intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
