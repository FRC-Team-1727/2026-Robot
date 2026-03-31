// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** An example command that uses an example subsystem. */
public class ForwardCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CommandSwerveDrivetrain m_Drivetrain;
  private double speed = 0;
  private SwerveRequest.FieldCentric drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ForwardCommand(CommandSwerveDrivetrain drivetrain) {
    m_Drivetrain = drivetrain;
    drive = new SwerveRequest.FieldCentric();
    System.out.println("foward");
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.withDesaturateWheelSpeeds(true)
        .withVelocityX(3.0)
        .withVelocityY(0.0);
    m_Drivetrain.setControl(drive);

    System.out.println("foward e");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
