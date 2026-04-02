// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_IntakeSubsystem;
  private final LEDSubsystem m_LedSubsystem;
  private double speed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_LedSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, ledSubsystem);
  }

  public IntakeCommand(IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, Double speed) {
    m_IntakeSubsystem = intakeSubsystem;
    m_LedSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed != 0) {
      m_IntakeSubsystem.setSpeed(speed);
    } else {
      m_IntakeSubsystem.setSpeed(IntakeConstants.intakeSpeed);
    }
    m_LedSubsystem.intake();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.passiveIntakeSpeed);
    m_LedSubsystem.PARTYMODE();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
