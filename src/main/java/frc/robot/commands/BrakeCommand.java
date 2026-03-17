// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class BrakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final CommandSwerveDrivetrain m_drivetrain;
  private final CommandXboxController joystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BrakeCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joysick) {
    m_drivetrain = drivetrain;
    this.joystick = joysick;
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
    m_drivetrain.setX();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (joystick.getLeftX() > .1 || joystick.getLeftX() < -.1) {
      return true;
    }
    if (joystick.getLeftY() > .1 || joystick.getLeftY() < -.1) {
      return true;
    }
    if (joystick.getRightX() > .1 || joystick.getRightX() < -.1) {
      return true;
    }
    return false;
  }
}
