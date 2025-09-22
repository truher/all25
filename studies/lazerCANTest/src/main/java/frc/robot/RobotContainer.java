// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.Util;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final Manipulator m_manipulator;

    public RobotContainer(TimedRobot100 robot) throws IOException {
      
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory logger = logging.rootLogger;

        final DriverControl driverControl = new DriverControlProxy(logger, async);

        final LoggerFactory sysLog = logger.name("Subsystems");
        CanBridge.runTCP();

        
        m_manipulator= new Manipulator(logger);
        m_manipulator.setDefaultCommand(m_manipulator.run(m_manipulator::stop));
        whileTrue(driverControl::a, m_manipulator.run(m_manipulator::intakeAlgae));
        whileTrue(driverControl::b, m_manipulator.run(m_manipulator::intakeSideways));
        whileTrue(driverControl::x, m_manipulator.run(m_manipulator::intakeCenter));
      }

  private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
