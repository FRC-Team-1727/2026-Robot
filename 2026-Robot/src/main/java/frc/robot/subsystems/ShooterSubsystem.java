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

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX shooterR = new TalonFX(ShooterConstants.kShooterRID);
  private TalonFX shooterL = new TalonFX(ShooterConstants.kShooterLID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    Slot0Configs configs = new Slot0Configs();
        
        configs.kS = ShooterConstants.kShooterS;
        configs.kV = ShooterConstants.kShooterV;
        configs.kP = ShooterConstants.kShooterP;
        configs.kI = ShooterConstants.kShooterI;
        configs.kD = ShooterConstants.kShooterD;

        shooterR.getConfigurator().apply(configs);
        shooterL.getConfigurator().apply(configs);
        CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

        configLimit.StatorCurrentLimit = 80;
        configLimit.SupplyCurrentLimit = 60;
        
        shooterR.setNeutralMode(NeutralModeValue.Coast);

        shooterL.setNeutralMode(NeutralModeValue.Coast);

        shooterR.getConfigurator().apply(configLimit);
        shooterL.getConfigurator().apply(configLimit);
       
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean shooterSpeed(){
    return shooterR.getVelocity().getValueAsDouble()>ShooterConstants.shooterRPSMinimum;
  }

  public void setSpeed(double speed){
    //  shooterL.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
    //  shooterR.setControl(m_request.withVelocity(-speed).withFeedForward(0.5));

         shooterL.setControl(new DutyCycleOut(speed));
          shooterR.setControl(new DutyCycleOut(-speed));


  }

  public double getSpeed(){
    return shooterR.getVelocity().getValueAsDouble();
  }
}
