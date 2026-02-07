// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  private TalonFX indexer = new TalonFX(IndexerConstants.kIndexerID);

  /** Creates a new ExampleSubsystem. */
  public IndexerSubsystem() {
    Slot0Configs configs = new Slot0Configs();
    CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

    configLimit.StatorCurrentLimit = 80;
    configLimit.SupplyCurrentLimit = 60;
  
    configs.kP = IndexerConstants.kIndexerP;
    configs.kI = IndexerConstants.kIndexerI;
    configs.kD = IndexerConstants.kIndexerD;

    indexer.getConfigurator().apply(configs);
    indexer.setNeutralMode(NeutralModeValue.Brake);
    indexer.getConfigurator().apply(configLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(double speed){
    indexer.setControl(new DutyCycleOut(speed));
  }
}
