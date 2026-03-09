package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Parking extends Component {
    public static class Params {
        public double TESTING_INC = 0.02;
        public double RETRACTED_POS = 0.075;
        public double EXTENDED_POS = 0.7;
        public double MIDDLE_POS = 0.5;
    }

    public static Params PARK_PARAMS = new Params();
    public enum ParkState {
        RETRACTED, EXTENDED, MIDDLE
    }
    public ServoImplEx parkLeftServo;
    public ServoImplEx parkRightServo;
    private ParkState parkState;

    public Parking(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        parkLeftServo = hardwareMap.get(ServoImplEx.class, "parkLeft");
        parkLeftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        parkRightServo = hardwareMap.get(ServoImplEx.class, "parkRight");
        parkRightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        setParkState(ParkState.RETRACTED);
    }

    public void setParkServoPosition(double position) {
        parkLeftServo.setPosition(position);
        parkRightServo.setPosition(position);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("===PARKING===");
        telemetry.addData("state", getParkState());
        telemetry.addData("servo pos (L|R)", parkLeftServo.getPosition() + " | " + parkRightServo.getPosition());
    }


    public ParkState getParkState() { return parkState; }
    public void setParkState(ParkState parkState) {
        this.parkState = parkState;
        switch (parkState) {
            case RETRACTED:
                setParkServoPosition(PARK_PARAMS.RETRACTED_POS);
                break;
            case EXTENDED:
                setParkServoPosition(PARK_PARAMS.EXTENDED_POS);
                break;
            case MIDDLE:
                setParkServoPosition(PARK_PARAMS.MIDDLE_POS);
                break;
        }
    }
    @Override
    public void update() {}
}
