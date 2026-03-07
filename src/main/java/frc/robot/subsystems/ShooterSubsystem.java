// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX shooterR = new TalonFX(ShooterConstants.kShooterRID);
  private TalonFX shooterL = new TalonFX(ShooterConstants.kShooterLID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    Slot0Configs configs = new Slot0Configs();
    FeedbackConfigs fConfigs = new FeedbackConfigs();
        configs.kS = ShooterConstants.kShooterS;
        configs.kV = ShooterConstants.kShooterV;
        configs.kP = ShooterConstants.kShooterP;
        configs.kI = ShooterConstants.kShooterI;
        configs.kD = ShooterConstants.kShooterD;
        fConfigs.RotorToSensorRatio=15/36;

        shooterR.getConfigurator().apply(configs);
        shooterL.getConfigurator().apply(configs);
        shooterR.getConfigurator().apply(fConfigs);
        shooterL.getConfigurator().apply(fConfigs);
        CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

        shooterL.getConfigurator().apply(fConfigs);

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

  public double getShooterPower(double distanceToTargetMeters) {
    // Constants - Adjust these to your robot's physical dimensions
    final double targetHeightMeters = 1.8288; // Height of the hoop
    final double shooterHeightMeters = 0.476758; // Height of your shooter exit
    final double angleDegrees = 62.0;
    final double g = 9.81;
      //RPM * Radius * 2PI / 60
    final double maxVelBottom = 30.0;
    final double maxVelTop = 30.0;
    final double maxVelocity = (maxVelBottom + maxVelTop)/2; // Max m/s your shooter can actually hit

    double x = distanceToTargetMeters;
    double y = targetHeightMeters - shooterHeightMeters;
    double theta = Math.toRadians(angleDegrees);

    // Projectile Motion Formula for Velocity
    double velocitySquared = (g * Math.pow(x, 2)) / 
        (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y));

    if (velocitySquared <= 0) return 0; // Target is physically unreachable

    double requiredVelocity = Math.sqrt(velocitySquared);

    // Normalize to a 0.0 - 1.0 range for motor output
    double power = requiredVelocity / maxVelocity;

    // Clamp the output between 0 and 1
    return Math.max(0, Math.min(1, power));
}
}
