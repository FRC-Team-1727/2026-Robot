// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.constants.OtherConstants.ShooterConstants;
import frc.robot.constants.OtherConstants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
  private TalonFX spindexer = new TalonFX(SpindexerConstants.kSpindexerID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);


  /** Creates a new ExampleSubsystem. */
  public SpindexerSubsystem() {
    Slot0Configs configs = new Slot0Configs();
    CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

    configLimit.StatorCurrentLimit = 80;
    configLimit.SupplyCurrentLimit = 40;
  
    configs.kS = SpindexerConstants.kSpindexerS;
    configs.kV = SpindexerConstants.kSpindexerV;
    configs.kP = SpindexerConstants.kSpindexerP;
    configs.kI = SpindexerConstants.kSpindexerI;
    configs.kD = SpindexerConstants.kSpindexerD;

    spindexer.getConfigurator().apply(configs);
    spindexer.setNeutralMode(NeutralModeValue.Coast);
    spindexer.getConfigurator().apply(configLimit);
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
    // spindexer.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
      spindexer.setControl(new DutyCycleOut(speed));
  }
}
