package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedAction implements Action {
    private final ElapsedTime timer;
    private final Action action;
    private final double maxTime;
    private boolean first = true;
    public Runnable onEnd;
    public TimedAction(Action action, double maxTime) {
        this.action = action;
        this.maxTime = maxTime;
        this.timer = new ElapsedTime();
        onEnd = () -> {};
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (first) {
            first = false;
            timer.reset();
        }

        boolean keepGoing = action.run(telemetryPacket);
        boolean outOfTime = timer.seconds() > maxTime;
        boolean done = !keepGoing || outOfTime;
        if(done)
            onEnd.run();
        return !done;
    }

    public TimedAction setEndFunction(Runnable onEnd) {
        TimedAction newAction = new TimedAction(action, maxTime);
        newAction.onEnd = onEnd;
        return newAction;
    }
}
