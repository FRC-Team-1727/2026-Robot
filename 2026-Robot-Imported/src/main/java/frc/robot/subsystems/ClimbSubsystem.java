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
import frc.robot.constants.OtherConstants.ClimbConstants;
import frc.robot.constants.OtherConstants.SpindexerConstants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX climb = new TalonFX(ClimbConstants.kClimbID);
  private boolean deployed;


  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    Slot0Configs configs = new Slot0Configs(); 
    CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

    configLimit.StatorCurrentLimit = 80;
    configLimit.SupplyCurrentLimit = 60;

    configs.kP = ClimbConstants.kClimbP;
    configs.kI = ClimbConstants.kClimbI;
    configs.kD = ClimbConstants.kClimbD;

    climb.getConfigurator().apply(configs);
    climb.setNeutralMode(NeutralModeValue.Brake);
    climb.getConfigurator().apply(configLimit);

    deployed = false;
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
    climb.setControl(new DutyCycleOut(speed));
  }


  public void switchClimbStatus(){
    deployed = !deployed; //changes the status of the climb from deployed and retracted
}

public boolean getClimbStatus(){
    return deployed;
}
}
