// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.SystemLights;
import frc.robot.util.HealthMonitor;
import frc.robot.util.HealthStatus;

public class DiagnosticLights extends Command {
    private List<RGBWColor> colors = new ArrayList<RGBWColor>();
    private int index;
    private final SystemLights cmdSystemLights;
    private final HealthMonitor cmdHealthMonitor;
    private final Timer timer;
    private final double timeout;

    public DiagnosticLights(SystemLights systemLights, double colorTimeout) {
        cmdSystemLights = systemLights;
        cmdHealthMonitor = HealthMonitor.getInstance();
        timer = new Timer();
        index = 0;
        timeout = colorTimeout;
        addRequirements(systemLights);
    }

    public DiagnosticLights(SystemLights systemLights) {
        this(systemLights, 3);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (cmdHealthMonitor.getSubsystemStatus("Elevator") == HealthStatus.ERROR) {
            colors.add(SystemLights.purple);
        }

        if (cmdHealthMonitor.getSubsystemStatus("Drivetrain") == HealthStatus.ERROR) {
            colors.add(SystemLights.red);
        }

        if (cmdHealthMonitor.getSubsystemStatus("Climber") == HealthStatus.ERROR) {
            colors.add(SystemLights.blue);
        }

        if (cmdHealthMonitor.getSubsystemStatus("AlgaeEndEffector") == HealthStatus.ERROR) {
            colors.add(SystemLights.white);
        }

        if (colors.size() == 0) {
            colors.add(SystemLights.green);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cmdSystemLights.setLEDs(colors.get(index % colors.size()));
        if (colors.size() > 1) { // Avoid divide by 0 and only cycle if there more than just one color
            if (!timer.isRunning()) {
                timer.start();
            }

            if (timer.advanceIfElapsed(timeout)) {
                index++;
                timer.reset();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        colors.clear();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return index == colors.size();
    }
}
