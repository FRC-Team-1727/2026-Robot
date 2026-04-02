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
    table.put(3.4036731719979793, 31.2578125);
    table.put(4.0192394256, 34.0937638125);
    table.put(5.164938449859619, 38.8828125);
    distanceS = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPS", getSpeed());
    SmartDashboard.putNumber("Shooter Additive", RobotContainer.getSpeedChange());
    SmartDashboard.putNumber("Interpolating", table.get(1.802564382553) + RobotContainer.getSpeedChange());
    SmartDashboard.putNumber("Formula", getShooterPower(distanceS));

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean shooterSpeed() {
    return shooterR.getVelocity().getValueAsDouble() > ShooterConstants.shooterRPSMinimum;
  }

  public void setSpeed(double speed) {
    shooterL.setControl(m_request.withVelocity(-speed).withFeedForward(0).withEnableFOC(false));
    shooterR.setControl(m_request.withVelocity(speed).withFeedForward(0).withEnableFOC(false));

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
    RPS += 0.65; // speedChange usually at 0.4 at Bethesda, set here at request of Dhruv
    // Will show 0.0 on Elastic with 0.4 applied

    distanceS = distanceToTargetMeters;
    return RPS - 2.0;
  }
}
