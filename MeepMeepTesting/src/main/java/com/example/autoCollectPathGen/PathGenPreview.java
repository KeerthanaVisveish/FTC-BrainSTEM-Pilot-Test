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
import java.util.List;

public class PathGenPreview extends JPanel
        implements MouseListener, MouseMotionListener, KeyListener {
    static final Path SAVE_FILE = Paths.get("MeepMeepTesting", "data", "pathGenSaveInfo.txt");

    static final int DRAW_SCALE = 4;
    static final double ROBOT_RADIUS = 8;
    static final double BALL_RADIUS = 2;
    static final double PATH_NODE_RADIUS = 2;
    static final double SELECTED_STROKE = 0.5;
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

    public PathGenPreview(String backgroundImagePath) {
        setPreferredSize(new Dimension(144 * DRAW_SCALE, 144 * DRAW_SCALE));
        setBackground(Color.WHITE);

        setFocusable(true);
        requestFocusInWindow();

        addMouseListener(this);
        addMouseMotionListener(this);
        addKeyListener(this);

        if (backgroundImagePath != null) {
            try {
                background = ImageIO.read(PathGenPreview.class.getResource(backgroundImagePath));
            } catch (IOException e) {
                System.err.println("Failed to load image: " + backgroundImagePath);
            }
        }

        loadFromFile();
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
                String[] parts = line.trim().split("\\s+");
                if (parts.length < 4) continue;

                double x = Double.parseDouble(parts[1]);
                double y = Double.parseDouble(parts[2]);
                double h = Double.parseDouble(parts[3]);

                if (parts[0].equals("ROBOT")) {
                    robot = new Pose2d(x, y, h);
                } else if (parts[0].equals("BALL")) {
                    balls.add(new Pose2d(x, y, h));
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    private void saveToFile() {
        try (BufferedWriter bw = Files.newBufferedWriter(SAVE_FILE)) {
            bw.write(String.format("ROBOT %.6f %.6f %.6f%n",
                    robot.position.x, robot.position.y, robot.heading.toDouble()));

            for (Pose2d ball : balls) {
                bw.write(String.format("BALL %.6f %.6f %.6f%n",
                        ball.position.x, ball.position.y, ball.heading.toDouble()));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;

        if (background != null)
            g2.drawImage(background, 0, 0, getWidth(), getHeight(), null);

        drawRobotAndBalls(g2);
        drawGeneratedPath(g2);
        drawBallsUsed(g2);

        g2.setColor(Color.BLACK);
        g2.fillRect(0, 0, 120, 400);
        g2.setColor(Color.WHITE);
        g2.drawString("Mode: " + createMode, 10, 15);
        g2.drawString("Simple Path: " + drawSimplifiedPath, 10, 35);
        g2.drawString("Robot: " + MathUtils.formatPose(robot), 10, 55);
        for (int i=0; i<balls.size(); i++) {
            g2.drawString("Ball: " + MathUtils.formatPose(balls.get(i)), 10, 75 + i * 20);
        }
    }

    private void drawRobotAndBalls(Graphics2D g2) {
        g2.setColor(Color.GRAY);
        int strokeSize = (int) (DRAW_SCALE * SELECTED_STROKE);
        g2.setStroke(new BasicStroke(strokeSize));
        drawPose(g2, robot, ROBOT_RADIUS);

        boolean selectedABall = selectedPoseIndex >= 0;
        boolean selectedRobot = selectedPoseIndex == -1;
        Pose2d selectedPose = null;
        if (selectedABall)
            selectedPose = balls.get(selectedPoseIndex);
        else if (selectedRobot)
            selectedPose = robot;

        for (int i=0; i<balls.size(); i++) {
            g2.setColor(Color.MAGENTA); // selected
            drawPosition(g2, balls.get(i).position, BALL_RADIUS, true);
        }

        g2.setColor(Color.WHITE);
        if (selectedABall)
            drawPosition(g2, selectedPose.position, BALL_RADIUS + SELECTED_STROKE, false);
        else if (selectedRobot) {
            drawPosition(g2, selectedPose.position, ROBOT_RADIUS + SELECTED_STROKE, false);
        }
    }
    private void drawGeneratedPath(Graphics2D g2) {
        Vector2d[] ballPositions = new Vector2d[balls.size()];
        for (int i=0; i<balls.size(); i++)
            ballPositions[i] = balls.get(i).position;
        ArrayList<Pose2d> poses = PathGeneration.getAutoCollectPathPoses(true, robot, ballPositions, 100, 3);
        if (poses == null)
            return;

        ArrayList<Pose2d> simplifiedPathPoses = PathGeneration.simplifyPathPoses(robot, poses);

        ArrayList<Pose2d> posesToDraw = new ArrayList<>(drawSimplifiedPath ? simplifiedPathPoses : poses);
        posesToDraw.add(0, robot);
        for (int i=0; i<posesToDraw.size(); i++) {
            Pose2d pose = posesToDraw.get(i);
            Pose2d next = i == posesToDraw.size() - 1 ? null : posesToDraw.get(i + 1);

            if (next != null) {
                Point p1 = fieldToDraw(pose.position);
                Point p2 = fieldToDraw(next.position);
                g2.setColor(Color.BLACK);
                g2.drawLine(p1.x, p1.y, p2.x, p2.y);
            }

            g2.setColor(Color.GRAY);
            drawPose(g2, pose, PATH_NODE_RADIUS);
        }
    }
    private void drawBallsUsed(Graphics2D g2) {
        if (PathGeneration.ballsUsed != null) {
            g2.setColor(Color.GREEN);
            for (Vector2d ball : PathGeneration.ballsUsed)
                drawPosition(g2, ball, 1, true);
        }
    }
    private Point fieldToDraw(Vector2d field) {
        return new Point((int) (field.x + 72.) * DRAW_SCALE, (int) (-field.y + 72.) * DRAW_SCALE);
    }
    private Vector2d drawToField(Point point) {
        return new Vector2d(1.*point.x / DRAW_SCALE - 72, -1.*point.y / DRAW_SCALE + 72);
    }
    private void drawPosition(Graphics2D g2, Vector2d position, double radiusField, boolean filled) {
        Point draw = fieldToDraw(position);
        int radius = (int) (radiusField * DRAW_SCALE);
        if (filled)
            g2.fillOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
        else
            g2.drawOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
    }
    private void drawPose(Graphics2D g2, Pose2d pose, double radiusField) {
        Point draw = fieldToDraw(pose.position);
        int radius = (int) (radiusField * DRAW_SCALE);
        g2.drawOval(draw.x - radius, draw.y - radius, radius * 2, radius * 2);
        int dx = (int) (pose.heading.component1() * radiusField * DRAW_SCALE);
        int dy = -(int) (pose.heading.component2() * radiusField * DRAW_SCALE);
        g2.drawLine(draw.x, draw.y, draw.x + dx, draw.y + dy);
    }

    @Override
    public void mousePressed(MouseEvent e) {
        requestFocusInWindow(); // ensure key events work
        Point mouse = e.getPoint();

        selectedPoseIndex = -2;
        for (int i=0; i<balls.size(); i++) {
            Pose2d pose = balls.get(i);
            Point point = fieldToDraw(pose.position);
            int radius = (int) (DRAW_SCALE * BALL_RADIUS);

            if (mouse.distance(point) <= radius) {
                createMode = CreateMode.BALL;
                selectedPoseIndex = i;
                repaint();
                return;
            }
        }

        int radius = (int) (ROBOT_RADIUS * DRAW_SCALE);
        if (fieldToDraw(robot.position).distance(mouse) <= radius) {
            selectedPoseIndex = -1; // select robot
        } else {
            // Add new circle if in BALL mode
            Vector2d creationPos = drawToField(mouse.getLocation());
            Pose2d creationPose = new Pose2d(creationPos.x, creationPos.y, 0);
            if (createMode == CreateMode.BALL) {
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

        Vector2d field = drawToField(new Point(e.getX(), e.getY()));

        if (selectedPoseIndex >= 0) {
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
        if (selectedPoseIndex == -2) return;

        boolean shouldRepaint = false;
        int keyCode = e.getKeyCode();
        if (keyCode == KeyEvent.VK_DELETE) {
            balls.clear();
            selectedPoseIndex = -2;
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_BACK_SPACE) {
            createMode = CreateMode.BALL;

            balls.remove(selectedPoseIndex);
            selectedPoseIndex = -2;
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_R) {
            createMode = CreateMode.ROBOT;
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_B) {
            createMode = CreateMode.BALL;
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_D) {
            robot = new Pose2d(robot.position.x, robot.position.y, robot.heading.toDouble() - Math.toRadians(1));
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_A) {
            robot = new Pose2d(robot.position.x, robot.position.y, robot.heading.toDouble() + Math.toRadians(1));
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_S) {
            drawSimplifiedPath = true;
            shouldRepaint = true;
        }
        else if (keyCode == KeyEvent.VK_C) {
            drawSimplifiedPath = false;
            shouldRepaint = true;
        }


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
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }
}
