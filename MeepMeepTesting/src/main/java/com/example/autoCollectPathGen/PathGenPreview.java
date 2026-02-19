package com.example.autoCollectPathGen;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

/*
controls
mouse: create points/drag points around
backspace: delete selected point
delete: delete all points
I: toggle debug info
WASD: pan camera
E/Q: zoom in/out
J/K: rotate robot right/left
P: toggle complex/simplified path
0/9: move current path point to draw robot pose at
 */
public class PathGenPreview extends JPanel
        implements MouseListener, MouseMotionListener, KeyListener {
    static final Path SAVE_FILE = Paths.get("MeepMeepTesting", "data", "pathGenSaveInfo.txt");

    static Vector2d bottomLeft = new Vector2d(0, 0);
    static Vector2d bottomLeftDraw = new Vector2d(0, 0);
    static double windowSize = 72;
    static int extraViewBuffer = 20;
    static boolean constrainBallsInsideField = true;
    static double drawScale = 4;
    static final double robotHitboxRadius = 8;
    static final double robotWidth = 13.5, robotLength = 16;
    static final double ballRadius = 2;
    static final double pathNodeRadius = 2;
    static final double strokeSize = 0.5;
    private final List<Pose2d> balls = new ArrayList<>();
    private Pose2d robot = new Pose2d(0, 0, 0);
    private int selectedPoseIndex = -2;
    private BufferedImage background;

    public enum CreateMode {
        ROBOT,
        BALL
    }
    private CreateMode createMode = CreateMode.BALL;
    private boolean drawSimplifiedPath = true;
    private int drawRobotNodeIndex = 0;
    private boolean drawInfo = false;

    public PathGenPreview(String backgroundImagePath) {
        loadFromFile();

        setPreferredSize(new Dimension(fieldToDrawSize(windowSize), fieldToDrawSize(windowSize)));
        setBackground(Color.WHITE);

        setFocusable(true);
        requestFocusInWindow();

        addMouseListener(this);
        addMouseMotionListener(this);
        addKeyListener(this);

        if (backgroundImagePath != null) {
            try {
                background = ImageIO.read(Objects.requireNonNull(PathGenPreview.class.getResource(backgroundImagePath)));
            } catch (IOException e) {
                System.err.println("Failed to load image: " + backgroundImagePath);
            }
        }
    }
    private void loadFromFile() {
        try {
            // Ensure data/ directory exists
            Files.createDirectories(SAVE_FILE.getParent());

            // Ensure file exists
            if (!Files.exists(SAVE_FILE)) {
                Files.createFile(SAVE_FILE);
                return;
            }
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }

        balls.clear();

        try (BufferedReader br = Files.newBufferedReader(SAVE_FILE)) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.trim().split(" ");

                if (parts[0].equals("BOTTOM_LEFT")) {
                    double x = Double.parseDouble(parts[1]);
                    double y = Double.parseDouble(parts[2]);
                    bottomLeft = new Vector2d(x, y);
                    bottomLeftDraw = bottomLeft;
                }
                else if (parts[0].equals("DRAW_SCALE"))
                    drawScale = Double.parseDouble(parts[1]);
                else if (parts[0].equals("WINDOW_SIZE"))
                    windowSize = Double.parseDouble(parts[1]);
                else if (parts[0].equals("ROBOT")) {
                    double x = Double.parseDouble(parts[1]);
                    double y = Double.parseDouble(parts[2]);
                    double h = Double.parseDouble(parts[3]);
                    robot = new Pose2d(x, y, h);
                } else if (parts[0].equals("BALL")) {
                    double x = Double.parseDouble(parts[1]);
                    double y = Double.parseDouble(parts[2]);
                    double h = Double.parseDouble(parts[3]);
                    balls.add(new Pose2d(x, y, h));
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    private void saveToFile() {
        try (BufferedWriter bw = Files.newBufferedWriter(SAVE_FILE)) {
            bw.write(String.format("BOTTOM_LEFT %.6f %.6f%n", bottomLeft.x, bottomLeft.y));
            bw.write(String.format("DRAW_SCALE %.6f%n", drawScale));
            bw.write(String.format("WINDOW_SIZE %.6f%n", windowSize));
            bw.write(String.format("ROBOT %.6f %.6f %.6f%n", robot.position.x, robot.position.y, robot.heading.toDouble()));
            for (Pose2d ball : balls) {
                bw.write(String.format("BALL %.6f %.6f %.6f%n", ball.position.x, ball.position.y, ball.heading.toDouble()));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        if (background != null) {
            double ratio = background.getWidth() / 144.;
            int x = (int) ((bottomLeftDraw.x + 72.) * ratio);
            int y = (int) ((bottomLeftDraw.y + 72.) * ratio);
            int w = (int) (windowSize * ratio);
            int h = (int) (windowSize * ratio);
            g2.drawImage(background, 0, 0, getWidth(), getWidth(), x, y, x + w, y + h, null);
        }

        g2.setColor(Color.LIGHT_GRAY);
        drawPosition(g2, new Vector2d(72, 72), PathGeneration.params.cornerBallDistance, false);
        drawPosition(g2, new Vector2d(72, -72), PathGeneration.params.cornerBallDistance, false);
        drawLine(g2, new Vector2d(72 - PathGeneration.params.backWallDistance, -72 + PathGeneration.params.classifierWallDistance), new Vector2d(72 - PathGeneration.params.backWallDistance, 72 - PathGeneration.params.classifierWallDistance));
        drawLine(g2, new Vector2d(-72 + PathGeneration.params.backWallDistance, 72 - PathGeneration.params.classifierWallDistance), new Vector2d(72 - PathGeneration.params.backWallDistance, 72 - PathGeneration.params.classifierWallDistance));
        drawLine(g2, new Vector2d(-72 + PathGeneration.params.backWallDistance, -72 + PathGeneration.params.classifierWallDistance), new Vector2d(72 - PathGeneration.params.backWallDistance, -72 + PathGeneration.params.classifierWallDistance));

        drawRobotAndBalls(g2);
        drawGeneratedPath(g2);
        drawBallsUsed(g2);

        if (drawInfo) {
            g2.setColor(Color.BLACK);
            g2.fillRect(0, 0, 120, 400);
            g2.setColor(Color.WHITE);
            g2.drawString("Mode: " + createMode, 10, 15);
            g2.drawString("Simple Path: " + drawSimplifiedPath, 10, 35);
            g2.drawString("Robot: " + MathUtils.formatPose(robot), 10, 55);
            for (int i = 0; i < balls.size(); i++) {
                g2.drawString("Ball: " + MathUtils.formatPose(balls.get(i)), 10, 75 + i * 20);
            }
        }
    }

    private void drawRobotAndBalls(Graphics2D g2) {
        g2.setColor(Color.GRAY);
        int strokeSize = fieldToDrawSize(PathGenPreview.strokeSize);
        g2.setStroke(new BasicStroke(strokeSize));
        drawRobot(g2, robot);
        Pose2d wallSafePose = PathGeneration.getWallSafePose(robot);
        drawRobot(g2, wallSafePose);
        boolean selectedABall = selectedPoseIndex >= 0;
        boolean selectedRobot = selectedPoseIndex == -1;
        Pose2d selectedPose = null;
        if (selectedABall)
            selectedPose = balls.get(selectedPoseIndex);
        else if (selectedRobot)
            selectedPose = robot;

        for (int i=0; i<balls.size(); i++) {
            g2.setColor(Color.MAGENTA); // selected
            drawPosition(g2, balls.get(i).position, ballRadius, true);
        }

        g2.setColor(Color.WHITE);
        if (selectedABall || selectedRobot)
            drawPosition(g2, selectedPose.position, ballRadius + PathGenPreview.strokeSize, false);
    }
    private void drawGeneratedPath(Graphics2D g2) {
        Vector2d[] ballPositions = new Vector2d[balls.size()];
        for (int i=0; i<balls.size(); i++)
            ballPositions[i] = balls.get(i).position;

        ArrayList<Pose2d> posesToDraw;
        if (drawSimplifiedPath)
            posesToDraw = PathGeneration.getSimplifiedAutoCollectPathPoses(true, robot, ballPositions, 100, 3);
        else
            posesToDraw = PathGeneration.getAutoCollectPathPoses(true, robot, ballPositions, 100, 3);
        if (posesToDraw == null)
            return;

        posesToDraw.add(0, robot);
        for (int i=0; i<posesToDraw.size(); i++) {
            Pose2d pose = posesToDraw.get(i);
            Pose2d next = i == posesToDraw.size() - 1 ? null : posesToDraw.get(i + 1);

            if (next != null) {
                Point p1 = fieldToDrawPosition(pose.position);
                Point p2 = fieldToDrawPosition(next.position);
                g2.setColor(Color.BLACK);
                g2.drawLine(p1.x, p1.y, p2.x, p2.y);
            }

            g2.setColor(Color.GRAY);
            drawPose(g2, pose, pathNodeRadius);
        }

        if (drawRobotNodeIndex < 0)
            drawRobotNodeIndex = 0;
        if (drawRobotNodeIndex >= posesToDraw.size())
            drawRobotNodeIndex = posesToDraw.size() - 1;
        if (drawRobotNodeIndex > 0)
            drawRobot(g2, posesToDraw.get(drawRobotNodeIndex));
    }
    private void drawBallsUsed(Graphics2D g2) {
        if (PathGeneration.ballsUsed != null) {
            g2.setColor(Color.GREEN);
            for (Vector2d ball : PathGeneration.ballsUsed)
                drawPosition(g2, ball, 1, true);
        }
    }
    private Point fieldToDrawPosition(Vector2d field) {
        return new Point((int) ((field.x - bottomLeftDraw.x) * drawScale), (int) ((-field.y - bottomLeftDraw.y) * drawScale));
    }
    private Vector2d drawToFieldPosition(Point point) {
        return new Vector2d(1.*point.x / drawScale + bottomLeftDraw.x, -1.*point.y / drawScale - bottomLeftDraw.y);
    }
    /*
    private Point fieldToDrawPosition(Vector2d field) {
        return new Point((int) ((field.x + 72.) * drawScale), (int) ((-field.y + 72.) * drawScale));
    }
    private Vector2d drawToFieldPosition(Point point) {
        return new Vector2d(1.*point.x / drawScale - 72, -1.*point.y / drawScale + 72);
    }
     */
    private int fieldToDrawSize(double size) {
        return (int) (size * drawScale);
    }
    private double drawToFieldSize(int size) {
        return 1.*size / drawScale;
    }
    private void drawPosition(Graphics2D g2, Vector2d position, double radiusField) {
        drawPosition(g2, position, radiusField, false);
    }
    private void drawPosition(Graphics2D g2, Vector2d position, double radiusField, boolean filled) {
        Point draw = fieldToDrawPosition(position);
        int radius = fieldToDrawSize(radiusField);
        if (filled)
            g2.fillOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
        else
            g2.drawOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
    }
    private void drawPose(Graphics2D g2, Pose2d pose, double radiusField) {
        Point draw = fieldToDrawPosition(pose.position);
        int radius = fieldToDrawSize(radiusField);
        g2.drawOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
        int dx = fieldToDrawSize(pose.heading.component1() * radiusField);
        int dy = -fieldToDrawSize(pose.heading.component2() * radiusField);
        g2.drawLine(draw.x, draw.y, draw.x + dx, draw.y + dy);
    }
    private void drawRobot(Graphics2D g2, Pose2d pose) {
        Vector2d position = pose.position;
        double halfW = robotWidth * 0.5;
        double halfL = robotLength * 0.5;

        Vector2d[] corners = new Vector2d[] {
                new Vector2d(halfL, -halfW), // fr
                new Vector2d(halfL, halfW), // fl
                new Vector2d(-halfL, halfW), // bl
                new Vector2d(-halfL, -halfW), // br
        };

        for (int j=0; j<corners.length; j++)
            corners[j] = GeometryUtils.robotVectorToFieldVector(corners[j], pose.heading.toDouble()).plus(position);

        for (int i=0; i<corners.length; i++)
            drawLine(g2, corners[i], corners[(i + 1) % 4]);
        Point center = fieldToDrawPosition(position);
        Point front = fieldToDrawPosition(corners[0].plus(corners[1]).div(2));
        g2.drawLine(center.x, center.y, front.x, front.y);
    }
    private void drawLine(Graphics2D g2, Vector2d p1, Vector2d p2) {
        Point start = fieldToDrawPosition(p1);
        Point end = fieldToDrawPosition(p2);
        g2.drawLine(start.x, start.y, end.x, end.y);
    }

    @Override
    public void mousePressed(MouseEvent e) {
        requestFocusInWindow(); // ensure key events work
        Point mouse = e.getPoint();

        selectedPoseIndex = -2;
        for (int i=0; i<balls.size(); i++) {
            Pose2d pose = balls.get(i);
            Point point = fieldToDrawPosition(pose.position);
            int radius = fieldToDrawSize(ballRadius);

            if (mouse.distance(point) <= radius) {
                createMode = CreateMode.BALL;
                selectedPoseIndex = i;
                repaint();
                return;
            }
        }

        int radius = fieldToDrawSize(robotHitboxRadius);
        if (fieldToDrawPosition(robot.position).distance(mouse) <= radius) {
            selectedPoseIndex = -1; // select robot
        } else {
            // Add new circle if in BALL mode
            Vector2d creationPos = drawToFieldPosition(mouse.getLocation());
            Pose2d creationPose = new Pose2d(creationPos.x, creationPos.y, 0);
            if (createMode == CreateMode.BALL) {
                if (constrainBallsInsideField) {
                    creationPose = new Pose2d(
                            Math.max(-72, Math.min(72, creationPose.position.x)),
                            Math.max(-72, Math.min(72, creationPose.position.y)),
                            0
                    );
                }
                balls.add(creationPose);
                selectedPoseIndex = balls.size() - 1;
            } else {
                robot = creationPose;
                selectedPoseIndex = -1;
            }
        }

        saveToFile();
        repaint();
    }
    @Override
    public void mouseDragged(MouseEvent e) {
        if (selectedPoseIndex == -2) return;

        Vector2d field = drawToFieldPosition(new Point(e.getX(), e.getY()));

        if (selectedPoseIndex >= 0) {
            if (constrainBallsInsideField)
                field = new Vector2d(
                        Math.max(-72, Math.min(72, field.x)),
                        Math.max(-72, Math.min(72, field.y))
                );
            Pose2d old = balls.get(selectedPoseIndex);
            balls.set(selectedPoseIndex, new Pose2d(field.x, field.y, old.heading.toDouble()));
        }
        else if (selectedPoseIndex == -1) {
            robot = new Pose2d(field.x, field.y, robot.heading.toDouble());
        }

        saveToFile();
        repaint();
    }
    @Override
    public void mouseReleased(MouseEvent e){}

    @Override
    public void keyPressed(KeyEvent e) {
        boolean shouldRepaint = false;
        int keyCode = e.getKeyCode();
        switch (keyCode) {
            case KeyEvent.VK_DELETE:
                balls.clear();
                selectedPoseIndex = -2;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_BACK_SPACE:
                if (selectedPoseIndex != -2) {
                    createMode = CreateMode.BALL;
                    balls.remove(selectedPoseIndex);
                    selectedPoseIndex = -2;
                    shouldRepaint = true;
                }
                break;
            case KeyEvent.VK_L:
                robot = new Pose2d(robot.position.x, robot.position.y, robot.heading.toDouble() - Math.toRadians(1));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_J:
                robot = new Pose2d(robot.position.x, robot.position.y, robot.heading.toDouble() + Math.toRadians(1));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_P:
                drawSimplifiedPath = !drawSimplifiedPath;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_9:
                drawRobotNodeIndex--;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_0:
                drawRobotNodeIndex++;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_I:
                drawInfo = !drawInfo;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_W:
                bottomLeft = bottomLeftDraw.plus(new Vector2d(0, -2));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_A:
                bottomLeft = bottomLeftDraw.plus(new Vector2d(-2, 0));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_S:
                bottomLeft = bottomLeftDraw.plus(new Vector2d(0, 2));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_D:
                bottomLeft = bottomLeftDraw.plus(new Vector2d(2, 0));
                shouldRepaint = true;
                break;
            case KeyEvent.VK_E:
                windowSize -= 4;
                windowSize = Math.max(24, Math.min(144, windowSize));
                bottomLeft = bottomLeftDraw.plus(new Vector2d(2, 2));
                drawScale = getWidth() / windowSize;
                shouldRepaint = true;
                break;
            case KeyEvent.VK_Q:
                windowSize += 4;
                windowSize = Math.max(24, Math.min(144, windowSize));
                bottomLeft = bottomLeftDraw.plus(new Vector2d(-2, -2));
                drawScale = getWidth() / windowSize;
                shouldRepaint = true;
                break;
        }

        bottomLeftDraw = new Vector2d(
                Math.max(-72 - extraViewBuffer, Math.min(72 + extraViewBuffer - windowSize, bottomLeft.x)),
                Math.max(-72 - extraViewBuffer, Math.min(72 + extraViewBuffer - windowSize, bottomLeft.y))
        );

        if (shouldRepaint) {
            saveToFile();
            repaint();
        }
    }

    @Override public void keyTyped(KeyEvent e) {}
    @Override public void keyReleased(KeyEvent e) {}

    // ---------------- Unused mouse methods ----------------

    @Override public void mouseClicked(MouseEvent e) {}
    @Override public void mouseEntered(MouseEvent e) {}
    @Override public void mouseExited(MouseEvent e) {}
    @Override public void mouseMoved(MouseEvent e) {}

    // ---------------- Main ----------------

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Circle Editor");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(new PathGenPreview("/com/example/autoCollectPathGen/img.png"));
            frame.setResizable(false);
            frame.pack();
            frame.setFocusable(true);
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
