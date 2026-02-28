package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;

@TeleOp(name="Turret Logger")
public class TurretLogger extends LinearOpMode {
    private final static ArrayList<double[]> turretPrevErrors = new ArrayList<>();
    private final static ArrayList<Boolean> turretWasOscillating = new ArrayList<>();
    public static void addInfo(double[] prevErrors, boolean isOscillating) {
        double[] newError = new double[prevErrors.length];
        for (int i=0; i<prevErrors.length; i++)
            newError[i] = prevErrors[i];
        turretPrevErrors.add(newError);
        turretWasOscillating.add(isOscillating);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        boolean reprint = true;

        while (opModeIsActive()) {
            if (gamepad1.start && gamepad1.backWasPressed()) {
                turretPrevErrors.clear();
                turretWasOscillating.clear();
                reprint = true;
            }
            if (reprint) {
                reprint = false;
                for (int i = 0; i < turretPrevErrors.size(); i++) {
                    telemetry.addLine(i + " | " + Arrays.toString(turretPrevErrors.get(i)) + " " + turretWasOscillating.get(i));
                }
                telemetry.update();
            }
        }
    }
}
