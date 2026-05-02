package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Parking extends Component {
    public static class Params {
        public double TESTING_INC = 0.001;
        public double RETRACTED_POS = .2;
        public double EXTENDED_POS = .71;
        public double testingPos = .2, testingInc = .001;
        public int leftLowerBound = 500, rightLowerBound = 500;
    }

    public static Params PARK_PARAMS = new Params();
    public enum ParkState {
        RETRACTED, EXTENDED, OFF, TESTING
    }
    public ServoImplEx parkLeftServo;
    public ServoImplEx parkRightServo;
    private ParkState parkState;

    public Parking(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        parkLeftServo = hardwareMap.get(ServoImplEx.class, "parkLeft");
        parkLeftServo.setPwmRange(new PwmControl.PwmRange(PARK_PARAMS.leftLowerBound, 2500));

        parkRightServo = hardwareMap.get(ServoImplEx.class, "parkRight");
        parkRightServo.setPwmRange(new PwmControl.PwmRange(PARK_PARAMS.rightLowerBound, 2500));

        setParkState(ParkState.RETRACTED);
    }

    public void setParkServoPosition(double position) {
        setLeftParkPosition(position);
        setRightParkPosition(position);
    }
    public void setLeftParkPosition(double position) {
        parkLeftServo.setPosition(position);
    }
    public void setRightParkPosition(double position) {
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
            case TESTING:
                setParkServoPosition(PARK_PARAMS.testingPos);
        }
    }
    @Override
    public void update() {
        switch (parkState) {
            case RETRACTED:
                setParkServoPosition(PARK_PARAMS.RETRACTED_POS);
                break;
            case EXTENDED:
                setParkServoPosition(PARK_PARAMS.EXTENDED_POS);
                break;
        }
    }
}
