// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FrontWheelsMoveCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CommandSwerveDrivetrain m_Drivetrain;
  private int startingRotation;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  public FrontWheelsMoveCommand(CommandSwerveDrivetrain drivetrain) {
    m_Drivetrain = drivetrain;
    startingRotation = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_Drivetrain.getModule(2).apply();
    if (startingRotation < 10) {
      m_Drivetrain.getModule(0).getSteerMotor().setControl(new PositionVoltage(.05));
      m_Drivetrain.getModule(1).getSteerMotor().setControl(new PositionVoltage(.05));
      startingRotation++;
      System.out.println("not rotate enough");
    } else if (startingRotation < 20) {
      m_Drivetrain.getModule(0).getDriveMotor().setControl(new DutyCycleOut(.5));
      m_Drivetrain.getModule(1).getDriveMotor().setControl(new DutyCycleOut(.5));
      System.out.println(" rotate");
      startingRotation++;
    } else {
      startingRotation = 0;
    }
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
