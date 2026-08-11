package org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.pilotAutoBuilder.helperClasses.PilotCommands;

/** Registers robot subsystem actions available to Brainstem Pilot JSON autos. */
public final class PilotCommandRegistry {
    private PilotCommandRegistry() {}

    public static void registerAll(BrainSTEMRobot robot) {
        PilotCommands.registerCommand("Collector", "Intake On", () -> packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.INTAKE);
            return false;
        });

        PilotCommands.registerCommand("Collector", "Intake Off", () -> packet -> {
            robot.collector.setIntakeState(Collector.IntakeState.OFF);
            return false;
        });

        PilotCommands.registerCommand("Shooter", "Shooter On", () -> packet -> {
            robot.shootingSystem.setShooterToGoalTargeting();
            robot.shootingSystem.setHoodToGoalTargeting();
            return false;
        });

        PilotCommands.registerCommand("Turret", "Track Turret", () -> packet -> {
            robot.shootingSystem.setTurretToGoalTargeting();
            return false;
        });

        PilotCommands.registerCommand("Transfer", "Engage Clutch", () -> packet -> {
            robot.collector.setClutchState(Collector.ClutchState.ENGAGED);
            return false;
        });

        PilotCommands.registerCommand("Transfer", "Disengage Clutch", () -> packet -> {
            robot.collector.setClutchState(Collector.ClutchState.DISENGAGED);
            return false;
        });

        PilotCommands.registerCommand("Transfer", "Flicker", () -> packet -> {
            robot.collector.setFlickerState(Collector.FlickerState.FULL_UP_DOWN);
            return false;
        });
    }
}