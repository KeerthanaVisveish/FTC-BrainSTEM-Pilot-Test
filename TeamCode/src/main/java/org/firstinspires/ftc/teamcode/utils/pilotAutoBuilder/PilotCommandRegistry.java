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
    }
}