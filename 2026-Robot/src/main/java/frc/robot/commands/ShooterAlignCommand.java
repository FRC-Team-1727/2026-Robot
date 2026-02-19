// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static edu.wpi.first.units.Units.Seconds;

import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.led.Animation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class ShooterAlignCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_Drivetrain;
  private final ShooterSubsystem m_ShooterSubsystem;
  // private ProfiledPIDController translationalPID;
  // private ProfiledPIDController rotationalPID;
  // private final CommandXboxController joystick;

  private final ConnectorXAnimate m_leds;
  //need limelights

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterAlignCommand(CommandSwerveDrivetrain drivetrain, ShooterSubsystem shooterSubsystem, ConnectorXAnimate leds) {
    m_Drivetrain = drivetrain;
    m_ShooterSubsystem = shooterSubsystem;
    m_leds = leds;    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_leds.leds.SetAnimation(Animation.Fill)
            .ForZone("front")
            .WithColor(new Color(new Color8Bit(0, 0, 255)))
            .WithDelay(Seconds.of(0))
            .Reverse(false)
            .RunOnce(false);
    //  translationalPID = new ProfiledPIDController(2, 0, 0,
    //             new TrapezoidProfile.Constraints(translationSpeedLim, translationAccelLim));
    //     rotationalPID = new ProfiledPIDController(6, 0, 0,
    //             new TrapezoidProfile.Constraints(rotationalSpeedLim, rotationalAccelLim));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //limelight auto align
    m_ShooterSubsystem.setSpeed(ShooterConstants.shooterSpeed);
    System.out.println(m_ShooterSubsystem.getSpeed());
     m_leds.leds.SetAnimation(Animation.Fill)
            .ForZone("front")
            .WithColor(new Color(new Color8Bit(0, 0, 255)))
            .WithDelay(Seconds.of(0))
            .Reverse(false)
            .RunOnce(false);
            System.out.println(m_leds.IsConnected());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_leds.leds.SetAnimation(Animation.RainbowRoll)
            .ForZone("front")
            .WithColor(new Color(new Color8Bit(255, 255, 255)))
            .WithDelay(Seconds.of(.5))
            .Reverse(false)
            .RunOnce(false);
    System.out.println(m_ShooterSubsystem.getSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (rotationalPID.atGoal() && translationalPID.atGoal()) || joystick.leftTrigger().getAsBoolean()
    //   || (!LimelightHelpers.getTV("limelight-left") && !LimelightHelpers.getTV("limelight-right"));
    return false;
  }
}
