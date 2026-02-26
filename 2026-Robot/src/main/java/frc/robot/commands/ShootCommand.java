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
public class ShootCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  private final SpindexerSubsystem m_SpindexerSubsystem;
  private final double shootSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, SpindexerSubsystem spindexerSubsystem, double shootSpeed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IndexerSubsystem = indexerSubsystem;
    m_SpindexerSubsystem = spindexerSubsystem;
    this.shootSpeed = shootSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, indexerSubsystem, spindexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_ShooterSubsystem.setSpeed(shootSpeed);
    // m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);


  //   if(m_ShooterSubsystem.shooterSpeed()){
  //     m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
  //     m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);
  // } else {
  //   m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
  //   m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
  // }
  
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
