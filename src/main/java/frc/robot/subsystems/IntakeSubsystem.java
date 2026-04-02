// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.IntakeConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);
  private TalonFX intake2 = new TalonFX(IntakeConstants.kIntakeID2);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    Slot0Configs configs = new Slot0Configs();
    CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

    configLimit.StatorCurrentLimit = 80;
    configLimit.SupplyCurrentLimit = 40;

    configLimit.StatorCurrentLimitEnable = true;
    configLimit.SupplyCurrentLimitEnable = true;

    configs.kS = IntakeConstants.kIntakeS;
    configs.kV = IntakeConstants.kIntakeV;
    configs.kP = IntakeConstants.kIntakeP;
    configs.kI = IntakeConstants.kIntakeI;
    configs.kD = IntakeConstants.kIntakeD;

    intake.getConfigurator().apply(configs);
    intake.setNeutralMode(NeutralModeValue.Coast);
    intake.getConfigurator().apply(configLimit);

    intake2.getConfigurator().apply(configs);
    intake2.setNeutralMode(NeutralModeValue.Coast);
    intake2.getConfigurator().apply(configLimit);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setSpeed(double speed) {
    // intake.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
    intake.setControl(new DutyCycleOut(speed));
    // intake2.setControl(new DutyCycleOut(speed));
  }
}
