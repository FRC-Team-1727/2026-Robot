// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.led.Animation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterAlignCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final ShooterSubsystem m_ShooterSubsystem;
  private SwerveRequest.FieldCentricFacingAngle turnCommand;
  SwerveRequest.FieldCentric drive;
  // private ProfiledPIDController translationalPID;
  // private ProfiledPIDController rotationalPID;
  private CommandXboxController joystick;
  private final RobotContainer robo;
  private final Rotation2d flip = new Rotation2d(Math.PI);
  private final LEDSubsystem m_LedSubsystem;
  private boolean isRed = false;
  Translation2d target = Hub.topCenterPointBlue.toTranslation2d();
  Rotation2d direction = null;
  float difference;

  // private final ConnectorX m_leds;
  private final PIDController pid;
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  // need limelights

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterAlignCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooterSubsystem,
      LEDSubsystem led, SwerveRequest.FieldCentric drive, RobotContainer robotContainer) {
    m_Drivetrain = drivetrain;
    m_ShooterSubsystem = shooterSubsystem;
    turnCommand = new SwerveRequest.FieldCentricFacingAngle();
    m_LedSubsystem = led;
    // m_leds = leds; // Use addRequirements() here to declare subsystem
    // dependencies.
    this.drive = drive;
    robo = robotContainer;
    this.joystick = robo.getJoystick();
    pid = new PIDController(1, 0, 1);
    pid.enableContinuousInput(-180, 180);
    addRequirements(drivetrain, shooterSubsystem);
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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isRed) {
      direction = (m_Drivetrain.getState().Pose.getTranslation()).minus(target)
          .getAngle();
    } else {
      direction = target.minus(m_Drivetrain.getState().Pose.getTranslation())
          .getAngle();
    }
    turnCommand.withDesaturateWheelSpeeds(true)
        .withHeadingPID(4.5, 0.0, 0.0)
        .withTargetDirection(direction)
        .withVelocityX(MaxSpeed * -joystick.getLeftY())
        .withVelocityY(MaxSpeed * -joystick.getLeftX());
    m_Drivetrain.setControl(turnCommand);

    difference = (float) m_Drivetrain.getState().Pose.getTranslation().getDistance(target);
    double power = m_ShooterSubsystem.getShooterPower(difference);
    m_ShooterSubsystem.setSpeed(power);

    // m_ShooterSubsystem.setSpeed(ShooterConstants.shooterSpeedClose);

    // System.out.println(m_ShooterSubsystem.getSpeed());
    // m_leds.leds.SetAnimation(Animation.Fill)
    // .ForZone("2")
    // .WithColor(new Color(new Color8Bit(0, 0, 255)))
    // .WithDelay(Seconds.of(0.5))
    // .Reverse(false)
    // .RunOnce(false);
    // //System.out.println(m_leds.IsConnected());

    // // m_leds.leds.SetAnimationSequence("front", "Test");
    double currentAngle = m_Drivetrain.getState().Pose.getRotation().getDegrees();
    double targetAngle = direction.getDegrees();
    double error = Math
        .abs(Rotation2d.fromDegrees(targetAngle).minus(Rotation2d.fromDegrees(currentAngle)).getDegrees());
    double speedError = Math.abs(power - m_ShooterSubsystem.getSpeed());
    if (error <= 5.0 && speedError <= 3.5) {
      m_LedSubsystem.aligned();
    } else {
      m_LedSubsystem.aligning();
      System.out.println(error + " " + speedError);
    }
  }

  // Called once the command ends or is interruRpted.
  @Override
  public void end(boolean interrupted) {
    // m_leds.leds.SetAnimation(Animation.RainbowRoll)
    // .ForZone("2")
    // .WithColor(new Color(new Color8Bit(255, 255, 255)))
    // .WithDelay(Seconds.of(.5))
    // .Reverse(false)
    // .RunOnce(false);
    // //System.out.println(m_ShooterSubsystem.getSpeed());
    // m_LedSubsystem.aligned();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double currentAngle =
    // m_Drivetrain.getState().Pose.getRotation().getDegrees();
    // double targetAngle = direction.getDegrees();
    // double error =
    // Math.abs(Rotation2d.fromDegrees(targetAngle).minus(Rotation2d.fromDegrees(currentAngle)).getDegrees());
    // if(error<=2.0){
    // return true;
    // }
    return false;
  }

}
