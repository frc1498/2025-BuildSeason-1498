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
import frc.robot.config.CoralIntakeConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

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

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator1 = new CommandXboxController(1);
    private final CommandXboxController operator2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorConfig elevatorConfig = new ElevatorConfig();
    public Elevator elevator = new Elevator(elevatorConfig);
 
    public final CoralIntakeConfig intakeConfig = new CoralIntakeConfig();
    public CoralIntake intake = new CoralIntake();

    public final WristConfig wristConfig = new WristConfig();
    public Wrist wrist = new Wrist(wristConfig);
 
    public Arm arm = new Arm();

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
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Elevator Simulation Practice Commands
        driver.povDown().onTrue(elevator.elevatorCoralL1());
        driver.povLeft().onTrue(elevator.elevatorCoralL2());
        driver.povUp().onTrue(elevator.elevatorCoralL3());
        //driver.povRight().onTrue(elevator.elevatorHome());
        driver.rightTrigger(0.1).whileTrue(elevator.elevatorPosition(() -> {return driver.getRightTriggerAxis() * 2;}));

        driver.leftBumper().whileTrue(intake.rollerSuck());

        //====================Operator Commands========================
        //Button Correlation Table
        //===========
        //Operator 1
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

        /*
        operator1.a().onTrue();
        operator1.b().onTrue();
        operator1.y().onTrue();
        operator1.x().onTrue();
        operator1.povDown().ontrue();
        operator1.povLeft().onTrue();
        operator1.povUp().onTrue();
        operator1.povRight().onTrue();
        operator1.back().onTrue();
        operator1.start().onTrue();
        operator1.rightBumper().onTrue();
        operator1.leftBumper().onTrue();
        operator1.leftStick().onTrue();
        operator1.rightStick().onTrue();

        operator2.a().onTrue();
        operator2.b().onTrue();
        operator2.y().onTrue();
        operator2.x().onTrue();
        operator2.povDown().ontrue();
        operator2.povLeft().onTrue();
        operator2.povUp().onTrue();
        operator2.povRight().onTrue();
        operator2.back().onTrue();
        operator2.start().onTrue();
        operator2.rightBumper().onTrue();
        operator2.leftBumper().onTrue();
        operator2.leftStick().onTrue();
        operator2.rightStick().onTrue();
*/

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
