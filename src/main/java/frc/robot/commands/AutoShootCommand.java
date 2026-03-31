// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.OtherConstants.IndexerConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.constants.OtherConstants.SpindexerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class AutoShootCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IndexerSubsystem m_IndexerSubsystem;
  private final SpindexerSubsystem m_SpindexerSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final LEDSubsystem m_LedSubsystem;
  private final CommandXboxController joystick;
  private final double shootSpeed;

  private boolean isRed = false;
  Translation2d target = Hub.topCenterPointBlue.toTranslation2d();
  float difference;
  private boolean intakeIncrease = false;
  private int counter = -1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoShootCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem,
      CommandXboxController joystick,
      double shootSpeed) {
    m_ShooterSubsystem = shooterSubsystem;
    m_IndexerSubsystem = indexerSubsystem;
    m_SpindexerSubsystem = spindexerSubsystem;
    m_IntakeSubsystem = intakeSubsystem;
    m_LedSubsystem = ledSubsystem;
    this.joystick = joystick;
    this.shootSpeed = shootSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, indexerSubsystem, spindexerSubsystem, intakeSubsystem, ledSubsystem);
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
    difference = (float) RobotContainer.getDrivetrain().getState().Pose.getTranslation().getDistance(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    counter++;
    // m_ShooterSubsystem.setSpeed(shootSpeed);
    difference = (float) RobotContainer.getDrivetrain().getState().Pose.getTranslation().getDistance(target);
    m_ShooterSubsystem.setSpeed(m_ShooterSubsystem.getShooterPower(difference) + .4);

    SmartDashboard.putNumber("Distance to Hub",
        (float) RobotContainer.getDrivetrain().getState().Pose.getTranslation().getDistance(target));

    m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);
    if (intakeIncrease) {
      m_IntakeSubsystem.setSpeed(IntakeConstants.intakeSlowShootSpeed + IntakeConstants.intakeChangeSpeed * counter);
      counter++;
    } else {
      m_IntakeSubsystem.setSpeed(IntakeConstants.intakeSlowShootSpeed + IntakeConstants.intakeChangeSpeed * counter);
      counter--;
    }

    if (counter > 20) {
      intakeIncrease = !intakeIncrease;
      counter = 0;
    }
    // m_Drivetrain.setX();

    // if(m_ShooterSubsystem.shooterSpeed()){
    // m_IndexerSubsystem.setSpeed(IndexerConstants.indexerSpeed);
    // m_SpindexerSubsystem.setSpeed(SpindexerConstants.spindexerSpeed);
    // } else {
    // m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
    // m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
    // }

    m_LedSubsystem.PARTYMODE();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.setSpeed(ShooterConstants.passiveShooterSpeed);
    m_IndexerSubsystem.setSpeed(IndexerConstants.passiveIndexerSpeed);
    m_SpindexerSubsystem.setSpeed(SpindexerConstants.passiveSpindexerSpeed);
    m_IntakeSubsystem.setSpeed(IntakeConstants.passiveIntakeSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
