package org.firstinspires.ftc.teamcode.subsystems.limelight.classifier;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LLParent;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;

import java.util.Arrays;

@Config
public class LimelightClassifier extends LLParent {
    public enum ReceiveState {
        NO_DATA,
        VALID,
        NO_CLASSIFIER,
        OCCLUDED_BALLS
    }
    public enum ReadState {
        OFF,
        READ
    }
    public static class Params {
        public double closeX = -12, closeY = 24, closeRadius = 24;
        public double farX = 60, farY = 12, farRadius = 6;
        public int numFramesPerRead = 3;
    }
    public static Params params = new Params();

    private boolean inValidClassifierRegion;
    private double[] pythonInputs;
    private double[] classifierDetectionOutput;
    private int[] numBalls; // numBalls[n] = the number of times n balls has been seen in the classifier from the limelight snapscript
    private int curFrameNumBalls;
    private int numFramesReading;
    private ReceiveState receiveState;
    private ReadState readState;
    public LimelightClassifier(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        classifierDetectionOutput = new double[0];
        numBalls = new int[10];
        numFramesReading = 0;
        pythonInputs = new double[2];
        receiveState = ReceiveState.NO_DATA;
        readState = ReadState.OFF;
    }

    public void resetForNewRead() {
        numBalls = new int[10];
        numFramesReading = 0;
    }

    public int getMostCommonNumBalls() {
        if (numFramesReading < params.numFramesPerRead)
            return -1;

        int mostCommonNumBalls = -1;
        int highestNum = 0;

        for (int i=0; i<numBalls.length; i++) {
            if (numBalls[i] > highestNum) {
                highestNum = numBalls[i];
                mostCommonNumBalls = i;
            }
        }
        return mostCommonNumBalls;
    }

    @Override
    public void update() {
        switch (readState) {
            case OFF:
                break;
            case READ:
                Pose2d robotPose = robot.drive.localizer.getPose();

                inValidClassifierRegion = inValidClassifierRegion(robotPose);
                if (!inValidClassifierRegion) {
                    setReadState(ReadState.OFF);
                    break;
                }

                pythonInputs = new double[2];
                pythonInputs[0] = BrainSTEMRobot.alliance == Alliance.RED ? 1 : -1; // red or blue alliance
                pythonInputs[1] = getCameraY();

                limelight.updatePythonInputs(pythonInputs);

                LLResult result = limelight.getLatestResult();
                classifierDetectionOutput = result.getPythonOutput();

                curFrameNumBalls = (int) classifierDetectionOutput[0];
                switch (curFrameNumBalls) {
                    case -2: receiveState = ReceiveState.OCCLUDED_BALLS; break;
                    case -1: receiveState = ReceiveState.NO_CLASSIFIER; break;
                    default: receiveState = ReceiveState.VALID; break;
                }
                if (receiveState == ReceiveState.VALID)
                    numBalls[curFrameNumBalls]++;

                numFramesReading++;
                if (numFramesReading >= params.numFramesPerRead)
                    setReadState(ReadState.OFF);
                break;
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("cur frame num balls", curFrameNumBalls);
        telemetry.addData("most common num balls", getMostCommonNumBalls());
        telemetry.addData("num ball results", Arrays.toString(numBalls));
        telemetry.addData("receive state", receiveState);
        telemetry.addData("in valid classifier region", inValidClassifierRegion);
        telemetry.addData("python inputs", Arrays.toString(pythonInputs));
        telemetry.addData("python outputs", Arrays.toString(classifierDetectionOutput));
    }

    public void addClassifierInfo(Canvas fieldOverlay) {
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(params.closeX, params.closeY, params.closeRadius);
        fieldOverlay.strokeCircle(params.farX, params.farY, params.farRadius);
    }

    private boolean inCloseZone(Pose2d robotPose) {
        return robotPose.position.x < 24;
    }
    private boolean inValidClassifierRegion(Pose2d robotPose) {
        boolean close = inCloseZone(robotPose);
        double tx = close ? params.closeX : params.farX;
        double ty = close ? params.closeY : params.farY;
        double tr = close ? params.closeRadius : params.farRadius;

        double dx = robotPose.position.x - tx;
        double dy = robotPose.position.y - ty;

        return dx * dx + dy * dy < tr * tr;
    }
    private double getCameraY() {
        Pose2d robotPose = robot.drive.localizer.getPose();
        Pose2d turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.curRelAngleRad);
        return Limelight.getLimelightPose(turretPose).position.y;
    }
    public Action readBallsInClassifier() {
        return new Action() {
            boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    resetForNewRead();
                    setReadState(ReadState.READ);
                }

                return readState == ReadState.READ;
            }
        };
    }
    public void setReadState(ReadState readState) {
        if (this.readState == readState)
            return;
        this.readState = readState;
        switch (readState) {
            case OFF:
                break;
            case READ:
                resetForNewRead();
        }
    }
}
/*
import cv2
import numpy as np
import math
import time

prev_update_time = time.perf_counter()

min_hue = 255
max_hue = 0
min_sat = 255
max_sat = 0
min_bright = 255
max_bright = 0

classifier_red_lb = (0, 100, 140)
classifier_red_ub = (10, 255, 255)
classifier_blue_lb = (90, 130, 50)
classifier_blue_ub = (130, 255, 140)
classifier_morph_open_size = 0
classifier_morph_close_size = 3

green_lb = (50, 35, 125)
green_ub = (70, 255, 230)
purple_lb1 = (150, 70, 60)
purple_ub1 = (180, 150, 230)
purple_lb2 = (0, 70, 60)
purple_ub2 = (10, 150, 230)

draw_info = True

def runPipeline(image, llrobot):
    global prev_update_time
    global draw_info
    global green_lb
    global green_ub
    global purple_lb1
    global purple_ub1
    global purple_lb2
    global purple_ub2

    current_update_time = time.perf_counter()
    dt = current_update_time - prev_update_time
    prev_update_time = current_update_time
    # print(dt)

    # is_red = llrobot[0] == 1
    # camera_y = llrobot[2]

    is_red = False
    camera_y = 24

    # PARAMETERS
    classifier_lb = classifier_red_lb if is_red else classifier_blue_lb
    classifier_ub = classifier_red_ub if is_red else classifier_blue_ub

    image = cv2.GaussianBlur(image, (3, 3), 0)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask image based on classifier========================
    classifier_contour_simplification_mult = 0.035
    valid_region_mask, classifier_col_mask, valid_region_bounds, successful = get_classifier_info(
        img_hsv,
        classifier_lb, classifier_ub,
        classifier_contour_simplification_mult,
        draw_info)
    # print(f"valid region bounds: {valid_region_bounds[0]}")
    if not successful:
        draw_side_text(image, "cannot find classifier", 760, 370)
        return contour_at_point(0, 0), image, [-1]

    image = cv2.bitwise_and(image, image, mask=valid_region_mask)

    # finding color contours===============================
    x1, y1, x2, y2 = valid_region_bounds
    cropped = img_hsv[y1:y2, :].copy()

    green_contours = get_contours(cropped, camera_y, green_lb, green_ub, None, None)
    purple_contours = get_contours(cropped, camera_y, purple_lb1, purple_ub1, purple_lb2, purple_ub2)
    # green_contours = []
    # purple_contours = []
    sample_x = 1000
    sample_y = 350
    show_sample_pixel(image, img_hsv, sample_x, sample_y, show_info=False)

    sorted_contours = []
    sorted_contours.extend(green_contours)
    sorted_contours.extend(purple_contours)
    sorted_contours.sort(key=centroid_x)
    if is_red:
        sorted_contours.reverse()

    if len(sorted_contours) == 0:
        draw_side_text(image, "no balls found", 830, 300)
        return contour_at_point(sample_x, sample_y), image, [0]

    # old rotation code
    # angle_rad = get_avg_angle_rad_between_sorted_contours(sorted_contours)
    # rounded_angle = int(100 * angle_rad * 180 / math.pi) / 100
    # cv2.putText(image, f"{rounded_angle}", (60, 60),
    #             cv2.FONT_HERSHEY_SIMPLEX,
    #             1.0, (255, 0, 255), 2, cv2.LINE_AA)
    # pivot = len(image[0]) * 0.5, len(image) * 0.5
    # rotated_contours = rotate_contours(pivot, angle_rad, sorted_contours)
    # rects = segment_contours(rotated_contours, camera_y)
    # rects = rotate_rects(pivot, angle_rad, rects)

    rects = segment_contours(sorted_contours, camera_y)
    total_num_balls = len(rects)

    # print(f"num balls: {total_num_balls}")
    if draw_info:
        dy = valid_region_bounds[1]
        for c in sorted_contours:
            c[:, 0, 1] += dy

        for i in range(len(rects)):
            x, y, w, h = rects[i]
            rects[i] = (x, y + dy, w, h)
        draw_combined_contours(image, sorted_contours)
        draw_rects(image, rects)

    draw_side_text(image, f"num balls: {total_num_balls}", 900, 250)

    return contour_at_point(sample_x, sample_y), image, [total_num_balls]


def get_classifier_info(img, lower_bounds, upper_bounds, contour_simplification_mult, draw_info):
    image = img.copy()
    width = len(image[0])
    height = len(image)
    crop_left = 200
    crop_right = width - 200
    triangle_height = 400
    triangle_width = 400

    cv2.rectangle(image, (0, 0), (crop_left, height), 0, -1)
    cv2.rectangle(image, (crop_right, 0), (width, height), 0, -1)

    middle = width / 2
    pts = np.array(
        [[middle - triangle_width * 0.5, height],
         [middle, height - triangle_height],
         [middle + triangle_width * 0.5, height]],
        dtype=np.int32
    ).reshape((-1, 1, 2))

    cv2.fillPoly(image, [pts], color=0)

    classifier_mask = cv2.inRange(image, lower_bounds, upper_bounds)

    global classifier_morph_open_size
    global classifier_morph_close_size
    if classifier_morph_open_size > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (classifier_morph_open_size, classifier_morph_open_size))
        classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_OPEN, kernel)
    if classifier_morph_close_size > 0:
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (classifier_morph_close_size, classifier_morph_close_size))
        classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_CLOSE, kernel2)

    contours, _ = cv2.findContours(classifier_mask,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if draw_info:
        classifier_mask = cv2.cvtColor(classifier_mask, cv2.COLOR_GRAY2BGR)

    mask = np.zeros((height, width), dtype=np.uint8)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        perimeter = cv2.arcLength(largest_contour, True)
        epsilon = contour_simplification_mult * perimeter
        approx = cv2.approxPolyDP(largest_contour, epsilon, closed=True)

        length, edge1, length2, edge2 = longest_2edges_from_approx(approx)
        p1 = edge1[0]
        p2 = edge1[1]

        if edge2 is not None:
            pp1 = edge2[0]
            pp2 = edge2[1]
            avgy1 = (p1[1] + p2[1]) * 0.5
            avgy2 = (pp1[1] + pp2[1]) * 0.5

            width2 = abs(pp2[0] - pp1[0])

            min_width_to_switch = 400

            if avgy2 < avgy1 and width2 > min_width_to_switch:
                p1 = pp1
                p2 = pp2

        # generating rectangle above classifier
        if p1[0] > p2[0]:
            p1, p2 = p2, p1
        r1x = p1[0]
        r1y = p1[1]
        r2x = p2[0]
        r2y = p2[1]

        dx = r2x - r1x
        dy = r2y - r1y

        r1x -= dx * 0.25
        r1y -= dy * 0.25
        r2x += dx * 0.25
        r2y += dy * 0.25

        min_width = 800
        if abs(r2x - r1x) < min_width:
            return mask, classifier_mask, [0, 0, 0, 0], False

        extra_space_down = 15
        r1y += extra_space_down
        r2y += extra_space_down

        extra_width = 0
        height_window = 90
        height_window += extra_space_down

        r1x -= extra_width
        r2x += extra_width

        r3x = r2x
        r3y = r2y - height_window
        r4x = r1x
        r4y = r1y - height_window

        if draw_info:
            cv2.polylines(
                classifier_mask,
                [approx],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2
            )
            cv2.circle(classifier_mask, (int(p1[0]), int(p1[1])), 20, (255, 0, 255), -1)
            cv2.circle(classifier_mask, (int(p2[0]), int(p2[1])), 20, (255, 0, 255), -1)
            cv2.circle(classifier_mask, (int(r1x), int(r1y)), 20, (0, 0, 255), -1)
            cv2.circle(classifier_mask, (int(r2x), int(r2y)), 20, (0, 0, 255), -1)
            cv2.circle(classifier_mask, (int(r3x), int(r3y)), 20, (0, 0, 255), -1)
            cv2.circle(classifier_mask, (int(r4x), int(r4y)), 20, (0, 0, 255), -1)

        pts = np.array(
            [[r1x, r1y],
             [r2x, r2y],
             [r3x, r3y],
             [r4x, r4y]],
            dtype=np.int32
        ).reshape((-1, 1, 2))

        cv2.fillPoly(mask, [pts], color=255)

        bounds = [
            0,
            max(0, int(min(r3y, r4y))),
            width,
            min(height, math.ceil(max(r1y, r2y)))
        ]
        return mask, classifier_mask, bounds, True
    return mask, classifier_mask, [0, 0, 0, 0], False


def get_contours(hsv, camera_y, lower_bounds, upper_bounds, lower_bounds2, upper_bounds2):
    img_threshold = cv2.inRange(hsv, lower_bounds, upper_bounds)
    if lower_bounds2 is not None and upper_bounds2 is not None:
        mask2 = cv2.inRange(hsv, lower_bounds2, upper_bounds2)
        img_threshold = cv2.bitwise_or(img_threshold, mask2)

    grow_size = (10, 10)
    shrink_size = (15, 15)
    grow = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, grow_size)
    shrink = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, shrink_size)
    img_threshold = cv2.dilate(img_threshold, grow)
    img_threshold = cv2.erode(img_threshold, shrink)

    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold,
                                   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # filter out specks
    # y=29.62963x+488.88889
    min_area = 29.62963 * camera_y + 488.88889

    min_accepted_area = -1
    filtered_contours = []
    for c in contours:
        area = cv2.contourArea(c)
        if area >= min_area:
            filtered_contours.append(c)
            if min_accepted_area < 0 or area < min_accepted_area:
                min_accepted_area = area
    contours = filtered_contours
    # print(f"min area: {int(min_area)} | min accepted: {min_accepted_area}")

    # filter out outliers by y value
    if len(contours) > 0:
        total_y = 0
        centroid_ys = []
        for c in contours:
            cy = centroid_y(c)
            centroid_ys.append(cy)
            total_y += cy
        mean_y = total_y / len(contours)

        max_dist_from_mean = 230
        filtered_contours = []
        for i in range(len(contours)):
            cy = centroid_ys[i]
            diff = abs(cy - mean_y)
            if diff <= max_dist_from_mean:
                filtered_contours.append(contours[i])
        contours = filtered_contours

    return contours


def get_avg_angle_rad_between_sorted_contours(sorted_contours):
    num_contours = len(sorted_contours)
    if num_contours == 0:
        return 0.

    if num_contours == 1:
        pts = sorted_contours[0].reshape(-1, 2).astype(np.float32)
        mean = np.mean(pts, axis=0)
        pts_centered = pts - mean  # center around (0, 0)
        _, eigenvectors = cv2.PCACompute(pts_centered, mean=None)
        vx, vy = eigenvectors[0]  # long direction
        return math.atan2(vy, vx)

    pairs = len(sorted_contours) // 2

    total_angle = 0.0
    n = len(sorted_contours)

    for i in range(pairs):
        fx, fy = centroid_xy(sorted_contours[i])
        lx, ly = centroid_xy(sorted_contours[n - 1 - i])
        total_angle += math.atan2(ly - fy, lx - fx)

    return total_angle / pairs


def rotate_contours(pivot, angle_rad, contours):
    cx, cy = pivot
    angle_deg = math.degrees(angle_rad)
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)

    rotated = []
    for c in contours:
        if c is None or len(c) == 0:
            continue

        c_float = c.astype(np.float32)  # (N,1,2)
        rc = cv2.transform(c_float, M)  # float32

        # Limelight-safe: drawContours prefers int32 point contours
        rc = np.round(rc).astype(np.int32)

        if rc.shape[0] > 0:
            rotated.append(rc)

    return rotated
    cx, cy = pivot
    angle_deg = math.degrees(angle_rad)
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)

    rotated = []
    for c in contours:
        if c is None or len(c) == 0:
            continue
        c_float = c.astype(np.float32)  # (N,1,2)
        rc = cv2.transform(c_float, M)
        if rc is not None and len(rc) > 0:
            rotated.append(rc)
    return rotated


def segment_contours(contours, camera_y):
    # single_ball_size = 100
    # y=2.53333x+55.88333
    single_ball_size = camera_y * 2.53333 + 55.88333
    another_ball_threshold = 0.8
    first_ball_size_mult = 1.1

    segmented_rects = []
    first_w = 0
    other_w = 0
    for i in range(len(contours)):
        contour = contours[i]
        x, y, w, h = cv2.boundingRect(contour)
        if i > 0:
            other_w += w
        else:
            first_w = w

        ball_size = single_ball_size
        if i == 0:
            ball_size *= first_ball_size_mult
        ratio = w / ball_size

        if ratio > 1 + another_ball_threshold:
            split_amount = int(ratio)
            if ratio % 1 > another_ball_threshold:
                split_amount += 1
            split_width = w / split_amount
            for i in range(split_amount):
                split_rect = x + i * split_width, y, split_width, h
                segmented_rects.append(split_rect)
        else:
            rect = x, y, w, h
            segmented_rects.append(rect)

    if len(contours) > 1:
        other_w /= len(contours) - 1
    # print(first_w, other_w)

    return segmented_rects


def rotate_rects(pivot, angle_rad, rects):
    cx, cy = pivot
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    rotated_rects = []

    for x, y, w, h in rects:
        # center of the rect
        rcx = x + w * 0.5
        rcy = y + h * 0.5

        # translate to pivot
        dx = rcx - cx
        dy = rcy - cy

        # rotate
        nx = dx * cos_a - dy * sin_a + cx
        ny = dx * sin_a + dy * cos_a + cy

        # convert back to top-left corner
        new_x = int(round(nx - w * 0.5))
        new_y = int(round(ny - h * 0.5))

        rotated_rects.append((new_x, new_y, w, h))

    return rotated_rects


# drawing functions========================
def show_sample_pixel(image, img_hsv, samplex, sampley, show_info):
    h, s, b = img_hsv[sampley, samplex]

    global min_hue
    global max_hue
    global min_sat
    global max_sat
    global min_bright
    global max_bright
    min_hue = min(h, min_hue)
    max_hue = max(h, max_hue)
    min_sat = min(s, min_sat)
    max_sat = max(s, max_sat)
    min_bright = min(b, min_bright)
    max_bright = max(b, max_bright)

    if show_info:
        draw_side_text(image, f"color: {h:3d} {s:3d} {b:3d}", 60, 360)
        draw_side_text(image, f"min col: {min_hue:3d} {min_sat:3d} {min_bright:3d}", 130, 400)
        draw_side_text(image, f"max col: {max_hue:3d} {max_sat:3d} {max_bright:3d}", 200, 400)


def draw_side_text(image, text, y, width):
    cv2.rectangle(image, (0, y), (width, y + 60), (0, 0, 0), -1)
    cv2.putText(
        image, text, (20, y + 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0, (0, 255, 0), 2,
        cv2.LINE_AA
    )


def draw_contours(image, green_contours, purple_contours):
    if len(green_contours) > 0:
        cv2.drawContours(image, green_contours, -1, (0, 255, 0), -1)
    if len(purple_contours) > 0:
        cv2.drawContours(image, purple_contours, -1, (200, 0, 200), -1)


def draw_combined_contours(image, contours):
    contours = [c for c in contours if c is not None and len(c) > 0 and c.shape[0] > 0]
    if len(contours) <= 0:
        return

    cv2.drawContours(image, contours, -1, (255, 255, 255), 2)


def draw_rects(image, rects):
    for i in range(len(rects)):
        x, y, w, h = rects[i]
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 0), 3)
        cv2.rectangle(image, (x - 5, y - 50), (x + 20, y), (0, 0, 0), -1)
        cv2.putText(image, f"{i}", (x, y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 255, 255), 2, cv2.LINE_AA)


# utility functions=========================
def centroid_x(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m10"] / M["m00"]


def centroid_y(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m01"] / M["m00"]


def centroid_xy(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0, 0.0
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy


def contour_at_point(x, y, size=6):
    # contour shape: (N, 1, 2) int32
    s = size
    pts = np.array([
        [x - s, y - s],
        [x + s, y - s],
        [x + s, y + s],
        [x - s, y + s],
    ], dtype=np.int32).reshape((-1, 1, 2))
    return pts


# returns length, (point1, point2)
def longest_2edges_from_approx(approx):
    pts = approx.reshape(-1, 2)  # (N, 2)
    n = len(pts)

    max_len = 0.0
    best_edge = None
    max_len2 = 0
    best_edge2 = None

    for i in range(n):
        p1 = pts[i]
        p2 = pts[(i + 1) % n]  # wrap around

        length = np.linalg.norm(p2 - p1)

        if length > max_len:
            max_len2 = length
            best_edge2 = best_edge
            max_len = length
            best_edge = (tuple(p1), tuple(p2))
        elif length > max_len2:
            max_len2 = length
            best_edge = (tuple(p1), tuple(p2))

    return max_len, best_edge, max_len2, best_edge2
 */
