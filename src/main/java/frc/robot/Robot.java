// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.temporal.ChronoUnit;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.HootAutoReplay;
import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;
import com.lumynlabs.domain.led.Animation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDSubsystem;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private ConnectorX m_leds = new ConnectorX();
    private LEDSubsystem m_LedSubsystem = new LEDSubsystem();

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer(m_leds, m_LedSubsystem);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        m_leds.leds.SetAnimation(Animation.RainbowRoll)
                .ForZone("2")
                .WithColor(new Color(new Color8Bit(255, 255, 255)))
                .WithDelay(Seconds.of(.5))
                .Reverse(false)
                .RunOnce(false);
    }

    @Override
    public void robotInit() {
        // Connect to the device on USB port 1
        boolean connected = m_leds.Connect(USBPort.kUSB2);
        for (int i = 0; i < 50; i++) {
            System.out.println("ConnectorX connected: " + connected);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        m_robotContainer.drivetrain.resetPose(new Pose2d());
        m_leds.leds.SetAnimation(Animation.Fill)
                .ForZone("2")
                .WithColor(new Color(new Color8Bit(0, 255, 0)))
                .WithDelay(Seconds.of(0.1))
                .RunOnce(false);
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            m_robotContainer.drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180.0));
        } else {
            m_robotContainer.drivetrain.seedFieldCentric(Rotation2d.fromDegrees(0.0));
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
