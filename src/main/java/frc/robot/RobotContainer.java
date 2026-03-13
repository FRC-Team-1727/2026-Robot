// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.lumynlabs.domain.config.ConfigBuilder;
import com.lumynlabs.domain.config.LumynDeviceConfig;
import com.lumynlabs.domain.config.NetworkType;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.led.Animation;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterAlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterAlignAutoCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    private final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    private final SpindexerSubsystem m_SpindexerSubsystem = new SpindexerSubsystem();
    private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    // private final ConnectorX m_leds;
    private final LEDSubsystem m_LedSubsystem;

    // private final LumynDevice mCx = new LumynDevice(3);
    private boolean aligning;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final RobotStateEstimator estimator = new RobotStateEstimator(drivetrain);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer(ConnectorX leds, LEDSubsystem m_leds) {

        configureBindings();
        configureNamedCommands();

        // m_leds = leds;
        // configureLEDS();

        m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);
        m_LedSubsystem = m_leds;

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("None", Commands.none());
        autoChooser.addOption("BM Gather Right", new PathPlannerAuto("BM Gather Right"));
        autoChooser.addOption("BL Gather", new PathPlannerAuto("BL Gather"));
        autoChooser.addOption("BM Gather Left", new PathPlannerAuto("BM Gather Left"));
        autoChooser.addOption("BR Gather", new PathPlannerAuto("BR Gather"));
        autoChooser.addOption("Move Forward", new PathPlannerAuto("Move Forward"));
        autoChooser.addOption("Test", new PathPlannerAuto("Test"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.17.27.11:5800/stream.mjpg");
        CameraServer.startAutomaticCapture(limelightFeed);

        m_LedSubsystem.PARTYMODE();

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> driveRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
                                                                                                          // negative Y
                                                                                                          // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // xbox Y = PS5 triangle

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.leftBumper().toggleOnTrue(new IntakeCommand(m_IntakeSubsystem));
        joystick.leftTrigger().whileTrue(new OuttakeCommand(m_IntakeSubsystem));
        // joystick.leftTrigger().onFalse(m_leds.leds.SetAnimation(Animation.RainbowRoll)
        // .ForZone("front")
        // .WithColor(new Color(new Color8Bit(255, 255, 255)))
        // .WithDelay(Seconds.of(.5))
        // .Reverse(false)
        // .RunOnce(false));
        joystick.rightBumper()
                .whileTrue(new ShooterAlignCommand(drivetrain, m_ShooterSubsystem, m_LedSubsystem, driveRequest, this)); // xbox
                                                                                                                         // X
                                                                                                                         // =
                                                                                                                         // PS5
                                                                                                                         // square
        joystick.rightTrigger().whileTrue(new ShootCommand(m_ShooterSubsystem, m_IndexerSubsystem, m_SpindexerSubsystem,
                m_IntakeSubsystem, drivetrain, m_LedSubsystem, ShooterConstants.shooterSpeedClose));
        // joystick.leftTrigger().whileTrue(new ShootCommand(m_ShooterSubsystem,
        // m_IndexerSubsystem, m_SpindexerSubsystem,
        // ShooterConstants.shooterSpeedTower));
        joystick.a().whileTrue(drivetrain.setX());
        joystick.povUp().onTrue(new ClimbCommand(m_ClimbSubsystem));

        joystick2.rightBumper().onTrue(new InstantCommand(
                () -> m_ShooterSubsystem.changeSpeed(ShooterConstants.shooterSpeedChange), m_ShooterSubsystem));
        joystick2.leftBumper().onTrue(new InstantCommand(
                () -> m_ShooterSubsystem.changeSpeed(0 - ShooterConstants.shooterSpeedChange), m_ShooterSubsystem));
        joystick2.y().onTrue(new InstantCommand(
                () -> m_ShooterSubsystem.resetSpeed(), m_ShooterSubsystem));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("Intake", new IntakeCommand(m_IntakeSubsystem).withTimeout(2.5));
        NamedCommands.registerCommand("Align",
                new ShooterAlignAutoCommand(drivetrain, m_ShooterSubsystem, m_LedSubsystem, driveRequest, this));
        NamedCommands.registerCommand("Shoot",
                new ShootCommand(m_ShooterSubsystem, m_IndexerSubsystem, m_SpindexerSubsystem, m_IntakeSubsystem,
                        drivetrain, m_LedSubsystem, ShooterConstants.shooterSpeedClose).withTimeout(2.5));
        NamedCommands.registerCommand("Climb", new ClimbCommand(m_ClimbSubsystem).withTimeout(2.5));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        // // Reset our field centric heading to match the robot
        // drivetrain.runOnce(() -> {
        // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        // drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180.0));
        // } else {
        // drivetrain.seedFieldCentric(Rotation2d.fromDegrees(0.0));
        // }
        // }),
        // // Then slowly drive forward (away from us) for 5 seconds.
        // drivetrain.applyRequest(() ->
        // driveRequest.withVelocityX(0.5)
        // .withVelocityY(0)
        // .withRotationalRate(0)
        // )
        // .withTimeout(5.0),
        // // Finally idle for the rest of auton
        // drivetrain.applyRequest(() -> idle)
        // );

        return autoChooser.getSelected();

    }

    // public void configureLEDS(){
    // Optional<LumynDeviceConfig> config=
    // m_leds.LoadConfigurationFromDeploy("config.json");
    // config.ifPresent(m_leds::ApplyConfiguration);
    // m_leds.leds.SetAnimation(Animation.Fill)
    // .ForZone("2")
    // .WithColor(new Color(new Color8Bit(0, 0, 255)))
    // .WithDelay(Units.Seconds.of(0))
    // .Reverse(false)
    // .RunOnce(false);
    // }

    public void configureAuto() {

    }

    public CommandXboxController getJoystick() {
        return joystick;
    }

    public void setAligning(boolean a) {
        aligning = a;
    }
}
