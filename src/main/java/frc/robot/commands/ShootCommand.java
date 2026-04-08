// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.OtherConstants.IndexerConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.constants.OtherConstants.SpindexerConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  private final SpindexerSubsystem m_SpindexerSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final LEDSubsystem m_LedSubsystem;
  private final CommandXboxController joystick;
  private final double shootSpeed;

  private boolean isRed = false;
  Translation2d target = Hub.topCenterPointBlue.toTranslation2d();
  float difference;
  Rotation2d direction = null;
  private boolean upToSpeed = false;

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private SwerveRequest.FieldCentricFacingAngle turnCommand;
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                    // second
                                                                                    // max angular velocity

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain drivetrain, LEDSubsystem ledSubsystem,
      CommandXboxController joystick,
      double shootSpeed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IndexerSubsystem = indexerSubsystem;
    m_SpindexerSubsystem = spindexerSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_Drivetrain = drivetrain;
    m_LedSubsystem = ledSubsystem;
    this.joystick = joystick;
    this.shootSpeed = shootSpeed;
    upToSpeed = false;

    turnCommand = new SwerveRequest.FieldCentricFacingAngle();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, indexerSubsystem, spindexerSubsystem, intakeSubsystem, ledSubsystem, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRed = DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    target = Hub.topCenterPointBlue.toTranslation2d();

    if (isRed) {
      target = Hub.topCenterPointRed.toTranslation2d();
    }
    difference = (float) m_Drivetrain.getState().Pose.getTranslation().getDistance(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_ShooterSubsystem.setSpeed(shootSpeed);
    System.out.println("pre everything");
    if (isRed) {
      direction = (m_Drivetrain.getState().Pose.getTranslation()).minus(target)
          .getAngle();
    } else {
      direction = target.minus(m_Drivetrain.getState().Pose.getTranslation())
          .getAngle();
      upToSpeed = RobotContainer.fromAlign();
    }
    // turnCommand.withDesaturateWheelSpeeds(true)
    // .withHeadingPID(4.5, 0.0, 0.0)
    // .withTargetDirection(direction)
    // .withVelocityX(MaxSpeed * -joystick.getLeftY())
    // .withVelocityY(MaxSpeed * -joystick.getLeftX());
    // System.out.println("turned");
    // m_Drivetrain.setControl(turnCommand);
    // System.out.println("turned2");

    boolean isMoving = (joystick.getLeftX() > .1 || joystick.getLeftX() < -.1 ||
        joystick.getLeftY() > .1 || joystick.getLeftY() < -.1 ||
        joystick.getRightX() > .1 || joystick.getRightX() < -.1);

    if (isMoving) {
      m_Drivetrain.setControl(driveRequest.withVelocityX(-joystick.getLeftY() * MaxSpeed)
          .withVelocityY(-joystick.getLeftX() * MaxSpeed)
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate));
    } else {
      m_Drivetrain.setX();
    }
    difference = (float) m_Drivetrain.getState().Pose.getTranslation().getDistance(target);
    double power = m_ShooterSubsystem.getShooterPower(difference);
    m_ShooterSubsystem.setSpeed(power);

    SmartDashboard.putNumber("Distance to Hub",
        (float) m_Drivetrain.getState().Pose.getTranslation().getDistance(target));
    double speedError = Math.abs(power - m_ShooterSubsystem.getSpeed());
    if (speedError <= 1.5 && !upToSpeed) {
      upToSpeed = true;
    }
    if (upToSpeed) {
      m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
      m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);
      m_IntakeSubsystem.setSpeed(IntakeConstants.shootingIntakeSpeed);
    }
    System.out.println(upToSpeed);
    m_LedSubsystem.PARTYMODE();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);
    m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
    m_IntakeSubsystem.setSpeed(IntakeConstants.passiveIntakeSpeed);
    upToSpeed = false;
    RobotContainer.changeAlign(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
