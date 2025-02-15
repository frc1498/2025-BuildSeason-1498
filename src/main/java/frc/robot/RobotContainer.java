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
import frc.pilotLib.utility.Selector;
import frc.robot.config.ArmConfig;
import frc.robot.config.CoralIntakeConfig;
import frc.robot.config.ElevatorConfig;
import frc.robot.config.WristConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LED;

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

    public EndEffector endEffector = new EndEffector();

    public LED leds = new LED();

    //Very important, the vision subsystem has to be created after the drivetrain.
    //The vision subsystem relies on creating a lambda that gets the drivetrain heading.
    public Vision vision = new Vision(() -> {return drivetrain.getPigeon2().getYaw().getValueAsDouble();});

    public Selector autoSelect = new Selector();

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

        /* SIMULATION PRACTICE COMMANDS
        //Elevator Simulation Practice Commands
        driver.povDown().onTrue(elevator.elevatorCoralL1());
        driver.povLeft().onTrue(elevator.elevatorCoralL2());
        driver.povUp().onTrue(elevator.elevatorCoralL3());
        //driver.povRight().onTrue(elevator.elevatorHome());
        driver.rightTrigger(0.1).whileTrue(elevator.elevatorPosition(() -> {return driver.getRightTriggerAxis() * 2;}));

        driver.leftBumper().whileTrue(intake.rollerSuck());

        driver.leftTrigger(0.1).onTrue(wrist.wristCoralL1()).onFalse(wrist.wristCoralL2());
        driver.y().onTrue(arm.armCoralL1()).onFalse(arm.armCoralL2());
        */

        driver.povDown().onTrue(endEffector.toCoralL1());
        driver.povLeft().onTrue(endEffector.toCoralL2());
        driver.povUp().onTrue(endEffector.toCoralL3());
        driver.povRight().onTrue(endEffector.toCoralStow());
        driver.rightBumper().onTrue(endEffector.toCoralSuck());

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



         //=======================Candle Control==========================
         /* 
        new JoystickButton(joy, Constants.BlockButton).onTrue(new RunCommand(m_candleSubsystem::setColors, m_candleSubsystem));
        new JoystickButton(joy, Constants.IncrementAnimButton).onTrue(new RunCommand(m_candleSubsystem::incrementAnimation, m_candleSubsystem));
        new JoystickButton(joy, Constants.DecrementAnimButton).onTrue(new RunCommand(m_candleSubsystem::decrementAnimation, m_candleSubsystem));

        new POVButton(joy, Constants.MaxBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 1.0));
        new POVButton(joy, Constants.MidBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0.3));
        new POVButton(joy, Constants.ZeroBrightnessAngle).onTrue(new CANdleConfigCommands.ConfigBrightness(m_candleSubsystem, 0));

        new JoystickButton(joy, Constants.VbatButton).onTrue(new CANdlePrintCommands.PrintVBat(m_candleSubsystem));
        new JoystickButton(joy, Constants.V5Button).onTrue(new CANdlePrintCommands.Print5V(m_candleSubsystem));
        new JoystickButton(joy, Constants.CurrentButton).onTrue(new CANdlePrintCommands.PrintCurrent(m_candleSubsystem));
        new JoystickButton(joy, Constants.TemperatureButton).onTrue(new CANdlePrintCommands.PrintTemperature(m_candleSubsystem));
        */

        // intake.coralgot.ontrue(new RunCommand(m_candleSubsystem::blinkLed, m_candleSubsystem));
        // need a trigger in the intake that this can check
      
        intake.isPartPresent.onTrue(leds.LEDsOn()).onFalse(leds.LEDsMode());  //Is a part in the intake OR in the gripper

        drivetrain.registerTelemetry(logger::telemeterize);
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
