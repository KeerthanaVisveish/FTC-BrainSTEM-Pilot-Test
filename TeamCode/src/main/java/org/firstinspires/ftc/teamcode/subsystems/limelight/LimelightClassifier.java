package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.Arrays;

@Config
public class LimelightClassifier extends LLParent {
    public static class Params {
        public double forwardDistFromTurret = 0;
        public double closeX = -12, closeY = 24, closeRadius = 12;
        public double farX = 60, farY = 12, farRadius = 6;
        public int numFramesPerRead = 3;
    }
    public static Params params = new Params();

    private boolean inValidClassifierRegion;
    private double[] pythonInputs;
    private double[] classifierDetectionOutput;
    private int[] numBalls; // numBalls[n] = the number of times n balls has been seen in the classifier from the limelight snapscript
    private int numFramesRunning;
    public LimelightClassifier(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);
        classifierDetectionOutput = new double[0];
        numBalls = new int[10];
        numFramesRunning = 0;
        pythonInputs = new double[3];
    }

    public void resetForNewRead() {
        numBalls = new int[10];
        numFramesRunning = 0;
    }

    public int getMostCommonNumBalls() {
        if (numFramesRunning < params.numFramesPerRead)
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
        Pose2d robotPose = robot.drive.localizer.getPose();

        // pausing limelight if cannot read classifier
        inValidClassifierRegion = inValidClassifierRegion(robotPose);

        pythonInputs = new double[3];
        pythonInputs[0] = BrainSTEMRobot.alliance == Alliance.RED ? 1 : -1; // red or blue alliance
        pythonInputs[1] = inCloseZone(robotPose) ? 1 : -1; // close or far zone
        pythonInputs[2] = 72 - Math.abs(getCameraY()); // distance from classifier

        limelight.updatePythonInputs(pythonInputs);

        LLResult result = limelight.getLatestResult();
        classifierDetectionOutput = result.getPythonOutput();

        int curFrameNumBalls = (int) classifierDetectionOutput[0];
        if(curFrameNumBalls != -1)
            numBalls[curFrameNumBalls]++;
        numFramesRunning++;
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("in valid classifier region", inValidClassifierRegion);
        telemetry.addData("python inputs", Arrays.toString(pythonInputs));
        telemetry.addData("python outputs", Arrays.toString(classifierDetectionOutput));
        telemetry.addData("num ball results", Arrays.toString(numBalls));
        telemetry.addData("most common num balls", getMostCommonNumBalls());
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
        int turretEncoder = robot.shootingSystem.getTurretEncoderRaw();
        Pose2d turretPose = ShootingMath.getTurretPose(robotPose, turretEncoder);
        double turretAngleRad = Turret.getTurretRelativeAngleRad(turretEncoder) + robotPose.heading.toDouble();
        double dy = params.forwardDistFromTurret * Math.sin(turretAngleRad);
        return turretPose.position.y + dy;
    }
}

/*
import cv2
import numpy as np
import math

def runPipeline(image, llrobot):
    is_red = llrobot[0] > 0
    is_close = llrobot[1] > 0
    dist = llrobot[2]

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask image based on classifier========================
    lower_bounds = (90,130,100)
    upper_bounds = (130,255,210)
    lower_bounds2 = None
    upper_bounds2 = None
    if is_red:
        lower_bounds = (160, 170, 150)
        upper_bounds = (180, 255, 255)
        lower_bounds2 = (0, 170, 150)
        upper_bounds2 = (10, 255, 255)

    valid_region_mask, successful = get_classifier_mask(img_hsv, is_close, lower_bounds, upper_bounds, lower_bounds2, upper_bounds2)
    show_sample_pixel(valid_region_mask, img_hsv, 612, 400)
    image = cv2.bitwise_and(image, image, mask=valid_region_mask)

    if not successful:
        return contour_at_point(0, 0), image, [ -1 ]

    # finding color contours===============================
    image = cv2.GaussianBlur(image, (3, 3), 0)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_contours = get_contours(img_hsv, is_close, (50, 35, 90), (80, 255, 255))
    purple_contours = get_contours(img_hsv, is_close, (140, 35, 90), (170, 255, 240))

    sorted_contours = []
    sorted_contours.extend(green_contours)
    sorted_contours.extend(purple_contours)
    sorted_contours.sort(key=centroid_x)

    total_area = 0
    areas = []
    for contour in sorted_contours:
        areas.append(cv2.contourArea(contour))
        total_area += areas[-1]

    #for i in range(len(sorted_contours)):
    #    print(f"{i}: {areas[i]}")

    # contour segmentation================================
    rects = []
    total_num_balls = 0
    if is_close:
        angle_rad = get_avg_angle_rad_between_sorted_contours(sorted_contours)
        rounded_angle = int(100 * angle_rad * 180 / math.pi) / 100
        cv2.putText(image, f"{rounded_angle}", (60, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 0), 2, cv2.LINE_AA)

        pivot = len(image[0]) * 0.5, len(image) * 0.5
        rotated_contours = rotate_contours(pivot, angle_rad, sorted_contours)

        rects = segment_contours(rotated_contours)
        rects = rotate_rects(pivot, angle_rad, rects)
        total_num_balls = len(rects)
    else:
        total_num_balls = 0

    #print(f"num balls: {total_num_balls}")
    draw_combined_contours(image, sorted_contours)
    draw_rects(image, rects)

    show_sample_pixel(image, img_hsv, 600, 400)
    #valid_region = cv2.cvtColor(valid_region_mask, cv2.COLOR_GRAY2BGR)
    return contour_at_point(600, 400), image, [ total_num_balls ]

def get_classifier_mask(image, is_close, lower_bounds, upper_bounds, lower_bounds2=None, upper_bounds2=None):
    width = len(image[0])
    height = len(image)
    crop_left = 250
    img = image.copy()
    crop_right = width - 250
    cv2.rectangle(img, (0, 0), (crop_left, height), 0, -1)
    cv2.rectangle(img, (crop_right, 0), (width, height), 0, -1)

    classifier_mask = cv2.inRange(img, lower_bounds, upper_bounds)
    if lower_bounds2 is not None and upper_bounds2 is not None:
        mask2 = cv2.inRange(img, lower_bounds2, upper_bounds2)
        classifier_mask = cv2.bitwise_or(classifier_mask, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_OPEN, kernel)
    classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(classifier_mask,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    classifier_mask = cv2.cvtColor(classifier_mask, cv2.COLOR_GRAY2BGR)

    mask = np.zeros((len(img), len(img[0])), dtype=np.uint8)
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        perimeter = cv2.arcLength(largest_contour, True)
        epsilon = 0.02 * perimeter
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
        r1x = p1[0]
        r1y = p1[1]
        r2x = p2[0]
        r2y = p2[1]
        if p1[0] > p2[0]:
            r1x = p2[0]
            r1y = p2[1]
            r2x = p1[0]
            r2y = p1[1]

        dx = r2x - r1x
        dy = r2y - r1y

        r1x -= dx * 0.1
        r1y -= dy * 0.1
        r2x += dx * 0.1
        r2y += dy * 0.1

        min_width = 800
        if abs(r2x - r1x) < min_width:
            print(abs(r2x - r1x))
            return mask, False

        extra_space_down = 60
        r1y += extra_space_down
        r2y += extra_space_down

        extra_width = 150 if is_close else 150
        height = 170 if is_close else 150

        r1x -= extra_width
        r2x += extra_width

        r3x = r2x
        r3y = r2y - height
        r4x = r1x
        r4y = r1y - height

        """
        cv2.polylines(
            classifier_mask,
            [approx],
            isClosed=True,
            color=(0, 255, 0),
            thickness=2
        )

        cv2.circle(classifier_mask, (int(r1x), int(r1y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r2x), int(r2y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r3x), int(r3y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r4x), int(r4y)), 20, (0, 0, 255), -1)
        """
        pts = np.array(
            [[r1x, r1y],
            [r2x, r2y],
            [r3x, r3y],
            [r4x, r4y]],
            dtype=np.int32
        ).reshape((-1, 1, 2))

        cv2.fillPoly(mask, [pts], color=255)

    return mask, True

def get_contours(hsv, is_close, lower_bounds, upper_bounds, lower_bounds2=None, upper_bounds2=None):
    img_threshold = cv2.inRange(hsv, lower_bounds, upper_bounds)
    #if lower_bounds2 is not None and upper_bounds2 is not None:
    #    mask2 = cv2.inRange(hsv, lower_bounds2, upper_bounds2)
    #    img_threshold = cv2.bitwise_or(img_threshold, mask2)

    grow_size = (10, 10) if is_close else (8, 8)
    shrink_size = (15, 15) if is_close else (8, 8)
    grow = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, grow_size)
    shrink = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, shrink_size)
    img_threshold = cv2.dilate(img_threshold, grow)
    img_threshold = cv2.erode(img_threshold, shrink)

    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # filter out specks
    min_area = 500 if is_close else 300

    filtered_contours = []
    for c in contours:
        if cv2.contourArea(c) >= min_area:
            filtered_contours.append(c)
    contours = filtered_contours

    # filter out outliers by y value
    if len(contours) > 0:
        total_y = 0
        centroid_ys = []
        for c in contours:
            cy = centroid_y(c)
            centroid_ys.append(cy)
            total_y += cy
        mean_y = total_y / len(contours)

        max_dist_from_mean = 230 if is_close else 200
        filtered_contours = []
        for i in range(len(contours)):
            cy = centroid_ys[i]
            diff = abs(cy - mean_y)
            if diff <= max_dist_from_mean:
                filtered_contours.append(contours[i])
        contours = filtered_contours

    return contours

def get_avg_angle_rad_between_sorted_contours(sorted_contours):
    pairs = len(sorted_contours) // 2
    if pairs == 0:
        return 0.0

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

        c_float = c.astype(np.float32)          # (N,1,2)
        rc = cv2.transform(c_float, M)          # float32

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

def segment_contours(contours):
    single_ball_size = 100
    another_ball_threshold = 0.8

    segmented_rects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        ratio = w / single_ball_size

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
def show_sample_pixel(image, img_hsv, samplex, sampley):
    h, s, b = img_hsv[sampley, samplex]
    cv2.rectangle(image, (0, 60), (360, 120), (0, 0, 0), -1)
    cv2.putText(
        image, f"color: {h} {s} {b}", (20, 100),
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
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 3)
        cv2.rectangle(image, (x - 5, y - 50), (x + 20, y), (0, 0, 0), -1)
        cv2.putText(image, f"{i}", (x, y-20),
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
