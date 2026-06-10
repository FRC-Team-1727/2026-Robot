// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.OtherConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX shooterR = new TalonFX(ShooterConstants.kShooterRID);
  private TalonFX shooterL = new TalonFX(ShooterConstants.kShooterLID);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  private double distanceS;
  private boolean formula = true;
  private String formulaString = "Formula";

  private final InterpolatingDoubleTreeMap table;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    Slot0Configs configs = new Slot0Configs();
    FeedbackConfigs fConfigs = new FeedbackConfigs();
    configs.kS = ShooterConstants.kShooterS;
    configs.kV = ShooterConstants.kShooterV;
    configs.kA = ShooterConstants.kShooterA;
    configs.kP = ShooterConstants.kShooterP;
    configs.kI = ShooterConstants.kShooterI;
    configs.kD = ShooterConstants.kShooterD;
    fConfigs.RotorToSensorRatio = 1.0;

    shooterR.getConfigurator().apply(configs);
    shooterL.getConfigurator().apply(configs);
    shooterR.getConfigurator().apply(fConfigs);
    shooterL.getConfigurator().apply(fConfigs);
    CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

    configLimit.StatorCurrentLimit = 80;
    configLimit.SupplyCurrentLimit = 40;

    configLimit.StatorCurrentLimitEnable = true;
    configLimit.SupplyCurrentLimitEnable = true;

    shooterR.setNeutralMode(NeutralModeValue.Coast);

    shooterL.setNeutralMode(NeutralModeValue.Coast);

    shooterR.getConfigurator().apply(configLimit);
    shooterL.getConfigurator().apply(configLimit);

    table = new InterpolatingDoubleTreeMap();
    table.put(1.802564382553, 25.93359375);
    table.put(2.04767760299, 27.425781225);
    table.put(2.61259273556, 28.060546375);
    table.put(2.7183398813842256, 29.37109275);
    table.put(2.99853897094, 29.61328425);
    table.put(3.170593738555908, 30.889475125);
    table.put(3.2199461460113525, 32.94921875);
    table.put(3.3115975856781, 33.61328125);
    table.put(3.39748825, 35.006859775);
    table.put(3.4820139408, 35.240234);
    table.put(3.552674298, 35.179687875);
    table.put(3.5915567874908447, 34.814453124);
    table.put(3.660079002371, 35.708984375);
    table.put(3.718740224838, 35.494762875);
    table.put(3.717987060546875, 36.02734375);
    table.put(4.24855232287695, 37.49023475);
    table.put(4.33591890335083, 37.099609375);
    table.put(4.378777560396, 38.498046875);
    table.put(4.94870710372928, 39.119140625);
    table.put(5.0353322029115, 39.326171875);
    table.put(5.103743076324463, 41.794921875);
    table.put(5.1390204429626465, 41.193359375);
    table.put(5.466902256011963, 43.2265625);
    distanceS = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (formula) {
      formulaString = "Formula";
    } else {
      formulaString = "Interpolating";
    }
    SmartDashboard.putNumber("Shooter RPS", getSpeed());
    SmartDashboard.putNumber("Shooter Additive", RobotContainer.getSpeedChange());
    SmartDashboard.putNumber("Interpolating", table.get(1.802564382553) + RobotContainer.getSpeedChange());
    SmartDashboard.putNumber("Formula", getShooterPower(distanceS));
    SmartDashboard.putString("Type Of Shooting", formulaString);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void changeShooting() {
    formula = !formula;
  }

  public boolean shooterSpeed() {
    return shooterR.getVelocity().getValueAsDouble() > ShooterConstants.shooterRPSMinimum;
  }

  public void setSpeed(double speed) {
    shooterL.setControl(
        m_request.withVelocity(-speed).withFeedForward(0).withEnableFOC(false));
    shooterR.setControl(
        m_request.withVelocity(speed).withFeedForward(0).withEnableFOC(false));

    double appliedVoltage = shooterL.getMotorVoltage().getValueAsDouble();
    double theoreticalVelocityRPS = appliedVoltage * ShooterConstants.kShooterV;
    // System.out.println(theoreticalVelocityRPS + " " +
    // shooterL.getVelocity().getValueAsDouble());

    // shooterL.setControl(new DutyCycleOut(-speed));
    // shooterR.setControl(new DutyCycleOut(speed));

    // System.out.println(shooterR.getDutyCycle().getValueAsDouble());
  }

  public double getSpeed() {
    return shooterR.getVelocity().getValueAsDouble();
  }

  public double interpolatingShooterPower(double distance) {
    distanceS = distance;
    return table.get(distance) + RobotContainer.getSpeedChange() - 2;
  }

  public double getShooterPower(double distanceToTargetMeters) {
    distanceS = distanceToTargetMeters;
    if (formula) {
      // Constants - Adjust these to your robot's physical dimensions
      final double targetHeightMeters = 1.8288; // Height of the hoop
      final double shooterHeightMeters = 0.476758; // Height of your shooter exit
      final double angleDegrees = 62.0;
      final double g = 9.81;

      double x = distanceToTargetMeters;
      double y = targetHeightMeters - shooterHeightMeters;
      double theta = Math.toRadians(angleDegrees);

      // Projectile Motion Formula for Velocity
      double velocitySquared = (g * Math.pow(x, 2)) /
          (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y));

      if (velocitySquared <= 0)
        return 0; // Target is physically unreachable

      double requiredVelocity = Math.sqrt(velocitySquared);

      double RPS = requiredVelocity / (Math.PI * .1016);
      RPS *= ShooterConstants.variableShootingMult;
      RPS += RobotContainer.getSpeedChange();
      RPS -= 1.5; // speedChange usually at 0.4 at Bethesda, set here at request of Dhruv
      // Will show 0.0 on Elastic with 0.4 applied
      return RPS;

    } else {
      return table.get(distanceToTargetMeters) + RobotContainer.getSpeedChange();
    }
  }
}
