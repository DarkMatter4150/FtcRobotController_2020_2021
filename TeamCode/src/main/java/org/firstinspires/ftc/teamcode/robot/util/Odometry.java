package org.firstinspires.ftc.teamcode.robot.util;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotPose2d;
import org.firstinspires.ftc.teamcode.robot.util.Encoder;

public class Odometry {
    public static final double TRACK_WIDTH = 14.75;
    public static final double FORWARD_OFFSET = -6.6;
    private static final double WHEEL_RADIUS = 0.944882;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 8192;

    //Make Sure RIGHT and LEFT Multiplier match in magnitude but they can vary in direction
    private static final double LEFT_MULTIPLIER = 1.0;
    private static final double RIGHT_MULTIPLIER = -1.0;
    private static final double CENTER_MULTIPLIER = -1.0;

    private double leftPrev;
    private double rightPrev;
    private double centerPrev;
    private double leftCurrent;
    private double rightCurrent;
    private double centerCurrent;
    private double leftDelta;
    private double rightDelta;
    private double centerDelta;

    private double currentX;
    private double currentY;
    private double currentTheta;

    private double prevX;
    private double prevY;
    private double prevTheta;

    private RobotPose2d posCurrent;
    private RobotPose2d posPrev;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder centerEncoder;

    private Telemetry telemetry;

    public Odometry(RobotPose2d startingPos, Encoder leftEncoder, Encoder rightEncoder, Encoder centerEncoder, Telemetry telemetry) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.centerEncoder = centerEncoder;
        this.telemetry = telemetry;
    }

    public void init() {
        leftPrev = 0;
        rightPrev = 0;
        centerPrev = 0;

        leftCurrent = 0;
        rightCurrent = 0;
        centerCurrent = 0;

        posPrev = new RobotPose2d(0,0,0);
        posCurrent = posPrev;

        prevX = 0;
        prevY = 0;
        prevTheta = 0;
        currentX = 0;
        currentY = 0;
        currentTheta = 0;
    }

    public void updateEncoders() {
        leftPrev = leftCurrent;
        rightPrev = rightCurrent;
        centerPrev = centerCurrent;

        leftCurrent = encoderTicksToInches(LEFT_MULTIPLIER * leftEncoder.getCurrentPosition());
        rightCurrent = encoderTicksToInches(RIGHT_MULTIPLIER * rightEncoder.getCurrentPosition());
        centerCurrent = encoderTicksToInches(CENTER_MULTIPLIER * centerEncoder.getCurrentPosition());

        setDeltas();
    }

    public void setDeltas() {
        leftDelta = leftCurrent - leftPrev;
        rightDelta = rightCurrent - rightPrev;
        centerDelta = centerCurrent - centerPrev;
    }

    public void update() {
        updateEncoders();
        double phi = (leftDelta - rightDelta) / TRACK_WIDTH;
        double[][] matrix1Data = {
                {Math.cos(prevTheta),-Math.sin(prevTheta),0},
                {Math.sin(prevTheta),Math.cos(prevTheta),0},
                {0,0,1}
        };
        double[][] matrix2Data = {
                {Math.sin(phi)/phi,(Math.cos(phi)-1)/phi,0},
                {(1-Math.cos(phi))/phi,Math.sin(phi)/phi,0},
                {0,0,1}
        };
        double[][] matrix3Data = {
                {(leftDelta+rightDelta)/2},
                {centerDelta-(FORWARD_OFFSET *phi)},
                {phi}
        };
        RealMatrix m1 = MatrixUtils.createRealMatrix(matrix1Data);
        RealMatrix m2 = MatrixUtils.createRealMatrix(matrix2Data);
        RealMatrix m3 = MatrixUtils.createRealMatrix(matrix3Data);

        RealMatrix r1 = m1.multiply(m2);
        RealMatrix r2 = r1.multiply(m3);
        prevX = currentX;
        prevY = currentY;
        prevTheta = currentTheta;

        if (r2.getColumnVector(0).getEntry(0) <=0 || r2.getColumnVector(0).getEntry(0) > 0) {
            currentX += r2.getColumnVector(0).getEntry(0);
            //telemetry.addData("DX: ", r2.getColumnVector(0).getEntry(0));
        }
        if (r2.getColumnVector(0).getEntry(1) <=0 || r2.getColumnVector(0).getEntry(1) > 0) {
            currentY += r2.getColumnVector(0).getEntry(1);
            //telemetry.addData("DY: ", r2.getColumnVector(0).getEntry(1));
        }
        if (r2.getColumnVector(0).getEntry(2) <=0 || r2.getColumnVector(0).getEntry(2) > 0) {
            currentTheta += r2.getColumnVector(0).getEntry(2);
            //telemetry.addData("DT: ", r2.getColumnVector(0).getEntry(2));
        }
        posCurrent = new RobotPose2d(posPrev.x + r2.getColumnVector(0).getEntry(0), posPrev.y + r2.getColumnVector(0).getEntry(1), posPrev.getHRad()+ r2.getColumnVector(0).getEntry(2));
        telemetry.addData("X: ", currentX);
        telemetry.addData("Y: ", currentY);
        telemetry.addData("H: ", Math.toDegrees(currentTheta));

        telemetry.addData("Left Enc: ", leftCurrent);
        telemetry.addData("Right Enc: ", rightCurrent);
        telemetry.addData("Center Enc: ", centerCurrent);
    }

    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


}
