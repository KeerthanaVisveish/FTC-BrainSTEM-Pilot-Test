package org.firstinspires.ftc.teamcode.utils.shootingMath;

import java.util.ArrayList;
import java.util.function.ToDoubleFunction;

public class ShootingMathNew {

  public final double g = 9.81;
  private final double findZeroesStepSize = .02, findZeroesErrorThreshold = .001;
  private final double[] timeOfFlightInterval = { .4, 3 };

  // parameters within god solve algorithm
  private final int godSolveIterations = 5;
  private final double speedTolerance = .00001;
  private final double speedHeuristicAlpha = .2;

  /**
   * computes lookahead and the ideal launch data assuming no velocity drop
   * @param exitPos
   * @param centerOfRotation
   * @param robotVelCm
   * @param robotAngularVel
   * @param goalPos
   * @param targetImpactAngleRad
   * @param lookAheadTime
   * @return ideal launch data and lookahead exit pos
   */
  public AnswerKeyPt1 godSolvePart1(Vector3d exitPos, Vector3d centerOfRotation, Vector3d robotVelCm, double robotAngularVel, Vector3d goalPos, double targetImpactAngleRad, double lookAheadTime) {
    // step 1: account for lookahead
    Vector3d currentRadius = exitPos.sub(centerOfRotation).to2D();
    Vector3d futureRelativeExitPos = rotateInXY(currentRadius, robotAngularVel * lookAheadTime);
    Vector3d lookAheadExitPos = centerOfRotation.add(robotVelCm.times(lookAheadTime)).add(futureRelativeExitPos).add(new Vector3d(0, 0, exitPos.z - centerOfRotation.z));

    Vector3d lookAheadTangentialVel = futureRelativeExitPos.perpInXY().times(robotAngularVel);
    Vector3d lookAheadVelAtExitPos = robotVelCm.add(lookAheadTangentialVel);

    Vector3d currentTangentialVel = currentRadius.perpInXY().times(robotAngularVel);
    Vector3d currentVelAtExitPos = robotVelCm.add(currentTangentialVel);

    // step 2: calculate ideal values
    LaunchVector idealBallLaunchVector = calculateBallLaunchData(lookAheadExitPos, goalPos, targetImpactAngleRad);
    if(!idealBallLaunchVector.valid)
      return new AnswerKeyPt1(lookAheadExitPos, lookAheadVelAtExitPos, currentVelAtExitPos);

    Vector3d idealBallLaunchVel = construct3DVector(idealBallLaunchVector);
    Vector3d idealShooterLaunchVel = idealBallLaunchVel.sub(lookAheadVelAtExitPos);
    LaunchVector idealShooterLaunchVector = decompose3DVector(idealShooterLaunchVel);

    return new AnswerKeyPt1(idealShooterLaunchVector, lookAheadExitPos, lookAheadVelAtExitPos, currentVelAtExitPos);
  }
  /**
   * part two of accounting for everything
   * takes data from part 1 and computers hood-dependent shooter speed heuristic
   * @param answerKeyPt1
   * @param goalPos
   * @param targetImpactAngleRad must be negative
   * @param shooterEncoderSpeed
   * @param shooterConversion
   * @return AnswerKey datatype containing info about solution (see AnswerKey class)
   */
  public AnswerKeyPt2 godSolvePart2(AnswerKeyPt1 answerKeyPt1, Vector3d goalPos, double targetImpactAngleRad, double shooterEncoderSpeed, ToDoubleFunction<Double> shooterConversion) {
    if(!answerKeyPt1.solutionExists)
      return new AnswerKeyPt2();

    // step 3: run heuristic to respond to velocity drop
    double shooterSpeedMps = shooterConversion.applyAsDouble(answerKeyPt1.launchVector.exitAng) * shooterEncoderSpeed; // initial guess at shooter speed
    LaunchVector estimatedLaunchVector = null;

    for(int i = 0; i < godSolveIterations; i++) {
      double prevShooterSpeedMps = shooterSpeedMps;
      estimatedLaunchVector = solve(answerKeyPt1.lookAheadExitPos, answerKeyPt1.lookAheadVelAtExitPos, goalPos, shooterSpeedMps, targetImpactAngleRad);
      if(!estimatedLaunchVector.valid)
        return new AnswerKeyPt2(prevShooterSpeedMps);
      shooterSpeedMps = shooterConversion.applyAsDouble(estimatedLaunchVector.exitAng) * shooterEncoderSpeed;
      shooterSpeedMps = prevShooterSpeedMps * speedHeuristicAlpha + shooterSpeedMps * (1 - speedHeuristicAlpha);

      double speedError = shooterSpeedMps - prevShooterSpeedMps;
      // System.out.println("error: " + speedError + ", exit ang: " + Math.toDegrees(estimatedLaunchData.exitAng) + ", turret ang: " + estimatedLaunchData.turretAng);

      if(Math.abs(speedError) < speedTolerance)
        break;
    }
    estimatedLaunchVector = new LaunchVector(shooterSpeedMps, estimatedLaunchVector.exitAng, estimatedLaunchVector.turretAng);
    return new AnswerKeyPt2(estimatedLaunchVector);
  }

  /**
   * solves for velocity vectory constrained by shooter speed that includes both a velocity responsive hood and the ability to shoot on the move
   * Z AXIS IS UP
   * @param exitPosMeters
   * @param robotVelAtExitPosMps
   * @param goalPosMeters
   * @param shooterSpeedMps
   * @param targetImpactAngleRad
   * @return launch data
   */
  public LaunchVector solve(Vector3d exitPosMeters, Vector3d robotVelAtExitPosMps, Vector3d goalPosMeters, double shooterSpeedMps, double targetImpactAngleRad) {
    Vector3d v = robotVelAtExitPosMps;
    double S = shooterSpeedMps;
    Vector3d r = goalPosMeters.sub(exitPosMeters);

    double a = .25 * g * g;
    double b = -g * v.z;
    double c = v.magSqrd() + r.z * g - S * S;
    double d = -2 * (r.x * v.x + r.y * v.y);
    double e = r.magSqrd();

    ToDoubleFunction<Double> timeOfFlight = t -> {
      double t2 = t * t;
      double t3 = t2 * t;
      double t4 = t2 * t2;
      return a * t4 + b * t3 + c * t2 + d * t + e;
    };

    ArrayList<Double> timeOfFlights = approxZeroes(timeOfFlight, timeOfFlightInterval);

    if(timeOfFlights.isEmpty()) {
      // System.out.println("NO SOLUTION FOUND; a: " + a + ", b: " + b + ", c: " + c + ", d: " + d + ", e: " + e);
      return new LaunchVector();
    }

    ArrayList<Double> impactAngles = new ArrayList<>();
    ArrayList<Vector3d> launchVectors = new ArrayList<>();
    
    for(double t : timeOfFlights) {
      double ballVelX = r.x/t;
      double ballVelY = r.y/t;
      double ballVelZ = r.z/t + .5*g*t;

      double ballImpactVelZ = ballVelZ - g * t;
      double ballImpactVelXY = Math.hypot(ballVelX, ballVelY);
      double impactAngle = Math.atan2(ballImpactVelZ, ballImpactVelXY);
      impactAngles.add(impactAngle);
      
      double launchVelX = ballVelX - v.x;
      double launchVelY = ballVelY - v.y;
      double launchVelZ = ballVelZ - v.z;

      launchVectors.add(new Vector3d(launchVelX, launchVelY, launchVelZ));
    }

    double minImpactAngleError = Double.MAX_VALUE;
    int desiredI = -1;
    for(int i = 0; i < impactAngles.size(); i++) {
      double error = Math.abs(impactAngles.get(i) - targetImpactAngleRad);
      if(error < minImpactAngleError) {
        minImpactAngleError = error;
        desiredI = i;
      }
    }
    if (desiredI == -1)
      return new LaunchVector();
    Vector3d launchVector = launchVectors.get(desiredI);
    double exitAngle = Math.atan2(launchVector.z, Math.hypot(launchVector.x, launchVector.y));
    double turretAngle = Math.atan2(launchVector.y, launchVector.x);

    return new LaunchVector(shooterSpeedMps, exitAngle, turretAngle);
  }

  private ArrayList<Double> approxZeroes(ToDoubleFunction<Double> f, double[] interval) {
    ArrayList<Double> zeroes = new ArrayList<>();

    // finding all intervals with zeros contained within them
    for(double outerX = interval[0]; outerX < interval[1]; outerX+=findZeroesStepSize) {
      double x0 = outerX;
      double x1 = Math.min(outerX + findZeroesStepSize, interval[1]);
      double f0Sign = Math.signum(f.applyAsDouble(x0));
      double f1Sign = Math.signum(f.applyAsDouble(x1));

      if(f0Sign == 0) {
        zeroes.add(x0);
        continue;
      }
      if(f1Sign == 0) {
        zeroes.add(x1);
        continue;
      }

      if(f0Sign == f1Sign)
        continue;
      
      double error = findZeroesStepSize;

      // finding more accurate approximation of zero
      do{
        double xMid = (x0 + x1) * .5;
        double fMidSign = Math.signum(f.applyAsDouble(xMid));
        
        if(f0Sign != fMidSign) {
          x1 = xMid;
          f1Sign = fMidSign;
        }
        else {
          x0 = xMid;
          f0Sign = fMidSign;
        }

        error /= 2;
      } while(error > findZeroesErrorThreshold);

      double approxX = (x0 + x1) * .5;
      zeroes.add(approxX);
    }
    return zeroes;
  }

  public static Vector3d construct3DVector(LaunchVector launchVector) {
    Vector3d topDownDir = new Vector3d(Math.cos(launchVector.turretAng), Math.sin(launchVector.turretAng), 0);
    double shootingAngleX = Math.cos(launchVector.exitAng);
    double shootingAngleY = Math.sin(launchVector.exitAng);
    Vector3d dir = topDownDir.times(shootingAngleX).add(new Vector3d(0, 0, shootingAngleY));
    return dir.times(launchVector.speed);
  }
  public LaunchVector decompose3DVector(Vector3d vec) {
    double base = Math.hypot(vec.x, vec.y);
    double v = Math.hypot(base, vec.z);
    double exitAng = Math.atan2(vec.z, base);
    double turretAng = Math.atan2(vec.y, vec.x);
    return new LaunchVector(v, exitAng, turretAng);
  }

  /**
   * @param exitPos 
   * @param goalPos
   * @param phi
   * @return launch data including shooter speed, exit angle, and turret angle
   */
  public LaunchVector calculateBallLaunchData(Vector3d exitPos, Vector3d goalPos, double phi) {
    double h = goalPos.z - exitPos.z;
    double d = Math.hypot(exitPos.x - goalPos.x, exitPos.y - goalPos.y);

    if(d == 0)
      return new LaunchVector();

    double exitAng = Math.atan(2 * h / d - Math.tan(phi)); 

    double num = g * d * d;
    double denom = 2 * (d * Math.tan(exitAng) - h) * Math.pow(Math.cos(exitAng), 2);

    if(denom <= 0)
      return new LaunchVector();

    double v = Math.sqrt(num / denom);

    double turretAng = Math.atan2(goalPos.y - exitPos.y, goalPos.x - exitPos.x);

    return new LaunchVector(v, exitAng, turretAng);
  }

  public Vector3d rotateInXY(Vector3d v, double a) {
    double cos = Math.cos(a);
    double sin = Math.sin(a);
    return new Vector3d(v.x * cos - v.y * sin, v.x * sin + v.y * cos, 0);
  }

  // helper functions to tune and test shooter encoder speed to exit speed conversion
  public double calculateExitSpeed(double dist, double height, double exitAngle) {
    double num = .5 * g * dist * dist;
    double denom = (Math.tan(exitAngle) * dist - height) * Math.pow(Math.cos(exitAngle), 2);
    if(denom < 0)
      throw new IllegalArgumentException("cannot calculate exit speed for dist=" + dist + ", height=" + height + ", and exit angle=" + exitAngle);
    return Math.sqrt(num / denom);
  }
  public double calculateShooterSpeed(double dist, double height, double exitAngle, ToDoubleFunction<Double> shooterConversion) {
    double desiredExitSpeed = calculateExitSpeed(dist, height, exitAngle);
    return desiredExitSpeed / shooterConversion.applyAsDouble(exitAngle);
  }

  public static void main(String[] args) {
    Vector3d exitPos = new Vector3d(0, 0, 0);
    Vector3d robotPos = new Vector3d(.1, 0, 0);
    Vector3d goalPos = new Vector3d(4.75, 4.07, 1.75);
    Vector3d robotVelAtExitPos = new Vector3d(0, 0, 0);
    double shooterSpeedEps = 9.156 * 2.2;
    double targetImpactAngle = Math.toRadians(-55);
    double lookAheadTime = 0;
    ToDoubleFunction<Double> shooterConversion = n -> -.005 * n + .5;

    ShootingMathNew shootingMath = new ShootingMathNew();
    System.out.println("results below");
    System.out.println("ideal solve: " + shootingMath.solve(exitPos, robotVelAtExitPos, goalPos, shooterSpeedEps * .5, targetImpactAngle));
    System.out.println();
    AnswerKeyPt1 godSolvePart1 = shootingMath.godSolvePart1(exitPos, robotPos, robotVelAtExitPos, 0, goalPos, targetImpactAngle, lookAheadTime);
    System.out.println("god solve pt1: " + godSolvePart1);
    if(godSolvePart1.solutionExists) {
      AnswerKeyPt2 godSolvePart2 = shootingMath.godSolvePart2(godSolvePart1, goalPos, targetImpactAngle, lookAheadTime, shooterConversion);
      System.out.println(godSolvePart2);
    }
  }
}
