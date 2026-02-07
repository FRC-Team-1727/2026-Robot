// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IndexerConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.constants.OtherConstants.SpindexerConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  private final SpindexerSubsystem m_SpindexerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, SpindexerSubsystem spindexerSubsystem) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IndexerSubsystem = indexerSubsystem;
    m_SpindexerSubsystem = spindexerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, indexerSubsystem, spindexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_ShooterSubsystem.setUse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_ShooterSubsystem.getUse()){
    m_ShooterSubsystem.setSpeed(ShooterConstants.shooterSpeed);
    if(m_ShooterSubsystem.shooterSpeed()){
      m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
      m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);
    }
  } else if (!m_ShooterSubsystem.getUse()){
    m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);
    m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);
    m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
