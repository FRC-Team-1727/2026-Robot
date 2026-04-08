// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.lumynlabs.devices.ConnectorX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.BackwardCommand;
import frc.robot.commands.BrakeCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.FixedShootCommand;
import frc.robot.commands.ForwardCommand;
import frc.robot.commands.FrontWheelsMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDeployCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterAlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterAlignAutoCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
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

        private LEDSubsystem m_LedSubsystem = new LEDSubsystem();

        private final boolean isRed;
        private Translation2d target;
        private Field2d autoField = new Field2d();
        private String newAutoName;
        private String autoName;
        private boolean shooter = false;
        private static boolean fromAlign = false;
        // private final ConnectorX m_leds;

        // private final LumynDevice mCx = new LumynDevice(3);
        private boolean aligning;

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final RobotStateEstimator estimator = new RobotStateEstimator(drivetrain);

        SendableChooser<Command> autoChooser = new SendableChooser<>();

        private static double speedChange = 0;

        public RobotContainer(ConnectorX leds) {

                configureBindings();
                configureNamedCommands();

                // m_leds = leds;
                // configureLEDS();

                m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);

                autoChooser = new SendableChooser<>();
                autoChooser.setDefaultOption("None", Commands.none());
                // autoChooser.addOption("BM Gather Right", new PathPlannerAuto("BM Gather
                // Right"));
                // autoChooser.addOption("BL Gather", new PathPlannerAuto("BL Gather"));
                // autoChooser.addOption("BM Gather Left", new PathPlannerAuto("BM Gather
                // Left"));
                // autoChooser.addOption("BM Trench Right", new PathPlannerAuto("BM Trench
                // Right"));
                // autoChooser.addOption("BR Gather", new PathPlannerAuto("BR Gather"));
                // autoChooser.addOption("Move Forward", new PathPlannerAuto("Move Forward"));
                // autoChooser.addOption("Test", new PathPlannerAuto("Test"));
                // autoChooser.addOption("BR Trench Gather", new PathPlannerAuto("BR Trench
                // Gather"));
                autoChooser.addOption("Right Trench Gather", new PathPlannerAuto("Right Trench Gather"));
                autoChooser.addOption("Left Trench Gather", new PathPlannerAuto("Left Trench Gather"));
                autoChooser.addOption("Middle Right Trench Gather", new PathPlannerAuto("Middle Right Trench Gather"));
                autoChooser.addOption("Middle Left Trench Gather", new PathPlannerAuto("Middle Left Trench Gather"));
                autoChooser.addOption("Right Bump Start Gather", new PathPlannerAuto("Right Bump Start Gather"));
                autoChooser.addOption("Middle Right Bump", new PathPlannerAuto("Middle Right Bump"));
                autoChooser.addOption("Right Trench Shoot Gather", new PathPlannerAuto("Right Trench Shoot Gather"));
                autoChooser.addOption("Middle Depot Collect", new PathPlannerAuto("Middle Depot Collect"));

                isRed = DriverStation.getAlliance()
                                .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
                target = Hub.topCenterPointBlue.toTranslation2d();

                if (isRed) {
                        target = Hub.topCenterPointRed.toTranslation2d();
                }

                SmartDashboard.putData("Auto Chooser", autoChooser);
                HttpCamera limelightFeed = new HttpCamera("limelight", "http://10.17.27.11:5800/stream.mjpg");
                CameraServer.startAutomaticCapture(limelightFeed);
                SmartDashboard.putData("Auto Field", autoField);

                m_LedSubsystem.PARTYMODE();

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> driveRequest
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                // forward
                                                                                                // with
                                                                                                // negative Y
                                                                                                // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
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

                joystick.leftBumper().toggleOnTrue(new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem));
                joystick.leftTrigger().whileTrue(new OuttakeCommand(m_IntakeSubsystem));
                // joystick.leftTrigger().onFalse(m_leds.leds.SetAnimation(Animation.RainbowRoll)
                // .ForZone("front")
                // .WithColor(new Color(new Color8Bit(255, 255, 255)))
                // .WithDelay(Seconds.of(.5))
                // .Reverse(false)
                // .RunOnce(false));
                joystick.rightBumper()
                                .whileTrue(new ShooterAlignCommand(drivetrain, m_ShooterSubsystem, m_LedSubsystem,
                                                driveRequest, this)); // xbox
                                                                      // X
                                                                      // =
                                                                      // PS5
                                                                      // square
                joystick.rightTrigger()
                                .whileTrue(new ShootCommand(m_ShooterSubsystem, m_IndexerSubsystem,
                                                m_SpindexerSubsystem,
                                                m_IntakeSubsystem, drivetrain, m_LedSubsystem, joystick,
                                                ShooterConstants.shooterSpeedClose));
                // joystick.rightTrigger().whileTrue(new RepeatCommand(new
                // BrakeCommand(drivetrain, joystick)));
                // joystick.leftTrigger().whileTrue(new ShootCommand(m_ShooterSubsystem,
                // m_IndexerSubsystem, m_SpindexerSubsystem,
                // ShooterConstants.shooterSpeedTower));
                joystick.a().onTrue(new BrakeCommand(drivetrain, joystick));
                joystick.povUp().onTrue(new ClimbCommand(m_ClimbSubsystem));
                joystick.x()
                                .toggleOnTrue(new FixedShootCommand(m_ShooterSubsystem, m_IndexerSubsystem,
                                                m_SpindexerSubsystem,
                                                m_IntakeSubsystem, drivetrain, m_LedSubsystem, joystick,
                                                ShooterConstants.outreachShooterSpeed));
                joystick.b().onTrue(new AutoShootCommand(m_ShooterSubsystem, m_IndexerSubsystem, m_SpindexerSubsystem,
                                m_IntakeSubsystem, m_LedSubsystem, joystick, MaxSpeed));

                joystick2.rightBumper().onTrue(new InstantCommand(
                                () -> changeSpeed(ShooterConstants.shooterSpeedChange)));
                joystick2.leftBumper().onTrue(new InstantCommand(
                                () -> changeSpeed(0 - ShooterConstants.shooterSpeedChange)));
                joystick2.y().onTrue(new InstantCommand(
                                () -> resetSpeed()));
                joystick2.rightTrigger()
                                .whileTrue(new RunCommand(() -> m_IntakeSubsystem
                                                .setSpeed(IntakeConstants.intakeFastShootSpeed)));
                joystick2.leftTrigger()
                                .whileTrue(new RunCommand(() -> m_IntakeSubsystem
                                                .setSpeed(IntakeConstants.intakeSlowShootSpeed)));
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand("Intake",
                                new IntakeCommand(m_IntakeSubsystem, m_LedSubsystem, -.8).withTimeout(3));
                NamedCommands.registerCommand("Align",
                                new ShooterAlignAutoCommand(drivetrain, m_ShooterSubsystem, m_LedSubsystem,
                                                driveRequest, this)
                                                .withTimeout(1));
                NamedCommands.registerCommand("Shoot",
                                new AutoShootCommand(m_ShooterSubsystem, m_IndexerSubsystem, m_SpindexerSubsystem,
                                                m_IntakeSubsystem,
                                                m_LedSubsystem, joystick, ShooterConstants.shooterSpeedClose)
                                                .withTimeout(6.5));
                NamedCommands.registerCommand("Climb", new ClimbCommand(m_ClimbSubsystem).withTimeout(2.5));
                NamedCommands.registerCommand("Intake Deploy",
                                new IntakeDeployCommand(m_IndexerSubsystem, m_ShooterSubsystem).withTimeout(1.4));
                NamedCommands.registerCommand("Rock", new ForwardCommand(drivetrain).withTimeout(.1)
                                .andThen(new BackwardCommand(drivetrain).withTimeout(.185)));
                NamedCommands.registerCommand("Brake", new BrakeCommand(drivetrain, joystick).withTimeout(.5));
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                // final var idle = new SwerveRequest.Idle();
                // return Commands.sequence(
                // // Reset our field centric heading to match the robot
                // drivetrain.runOnce(() -> {
                // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                // drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180.0));
                // } else {
                // drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180.0));
                // }
                // });
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
                if (isRed) {

                        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
                } else {
                        drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
                }

                return autoChooser.getSelected();

        }

        public static CommandSwerveDrivetrain getDrivetrain() {
                return drivetrain;
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

        public void periodic() {
                // if (m_ShooterSubsystem.getCurrentCommand().getName() != null
                // &&
                // m_ShooterSubsystem.getCurrentCommand().getName().equals("FixedShootCommand"))
                // {
                // shooter = true;
                // } else {
                // shooter = false;
                // }

                SmartDashboard.putBoolean("Fixed Shoot Toggle", joystick.x().getAsBoolean());
        }

        public void configureAuto() {

        }

        public CommandXboxController getJoystick() {
                return joystick;
        }

        public void setAligning(boolean a) {
                aligning = a;
        }

        public void changeSpeed(double s) {
                speedChange += s;
        }

        public void resetSpeed() {
                speedChange = 0;
        }

        public static double getSpeedChange() {
                return speedChange;
        }

        public static boolean fromAlign() {
                return fromAlign;
        }

        public static void changeAlign(boolean a) {
                fromAlign = a;
        }
}
