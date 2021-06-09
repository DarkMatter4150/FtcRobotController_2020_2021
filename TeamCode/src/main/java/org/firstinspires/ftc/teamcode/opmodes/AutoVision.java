/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.OptionalDouble;

@Autonomous
public class AutoVision extends LinearOpMode
{
    public static double SPEED = .5;
    public static double PVALUE = .05;
    Robot robot;
    public boolean willResetIMU = true;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    Pose startingPose = new Pose(0, 0, 0);
    Pose currentPose = new Pose().copy(startingPose);

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;


    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //Init
        ArrayList<Integer> values = new ArrayList<Integer>();
        robot = new Robot(this, true);
        robot.initIMU();
        robot.setClaw(false);
        

        waitForStart();
        ElapsedTime autoTimer = new ElapsedTime();


        while (opModeIsActive() && autoTimer.milliseconds() <= 1000)
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Position Integer", pipeline.positionInt);
            telemetry.update();
            values.add(pipeline.positionInt);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

        }

        double average = 0;

        try {
            average = values.stream().mapToDouble(a -> a).average().orElse(-1);
        }
        catch(Exception e) {
            telemetry.addData("Error Caught: ", "1");
        }


        telemetry.addData("Average", average);
        telemetry.update();
        sleep(1000);
/*
        if(average == -1){
            //run park auto
        }else if (average > 2.5){
            fourRingAuto();
        }else if (average > 0.5){
            oneRingAuto();
        } else {
            zeroRingAuto();
        } */

        if(pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR){
            fourRingAuto();
        }else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE){
            oneRingAuto();
        }else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE){
            zeroRingAuto();
        } else {
            telemetry.addData("No Rings: ", 0);
            fourRingAuto();
        }

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(465,    190);

        static final int REGION_WIDTH = 170;
        static final int REGION_HEIGHT = 120;

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;
        private volatile Integer positionInt = null;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
                positionInt = 4;

            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
                positionInt = 1;
            }else{
                position = RingPosition.NONE;
                positionInt = 0;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void fourRingAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,112)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.setArmPower(-1);
        sleep(1250);
        robot.setArmPower(0);
        robot.setClaw(true);
        robot.setArmPower(1);
        sleep(1000);
        robot.setArmPower(0);


        path = new Path().addPoint(new PathPoint(-6, 50)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(4, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);

        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(-1, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);


        path = new Path().addPoint(new PathPoint(-6, 63)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(-84, Angle.AngleType.NEG_180_TO_180_HEADING),.9, this);
    }

    public void oneRingAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(-12,82)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.3, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.setArmPower(-1);
        sleep(1250);
        robot.setArmPower(0);
        robot.setClaw(true);
        robot.setArmPower(1);
        sleep(1000);
        robot.setArmPower(0);


        path = new Path().addPoint(new PathPoint(-6, 50)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(4, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);

        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(-1, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);


        path = new Path().addPoint(new PathPoint(-6, 63)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(-84, Angle.AngleType.NEG_180_TO_180_HEADING),.9, this);
    }

    public void zeroRingAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,71)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.setArmPower(-1);
        sleep(1250);
        robot.setArmPower(0);
        robot.setClaw(true);
        robot.setArmPower(1);
        sleep(1000);
        robot.setArmPower(0);


        path = new Path().addPoint(new PathPoint(-6, 50)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(4, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);

        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.driveController.rotateRobot(new Angle(-1, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setBoxLifter(true);
        robot.setFlywheelPower((float)-0.85);
        sleep(1500);
        robot.setPusherThing(true);
        sleep(1000);
        robot.setFlywheelPower(0);
        robot.setPusherThing(false);
        sleep(2000);


        path = new Path().addPoint(new PathPoint(-6, 63)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1*Math.sqrt(2), 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(-84, Angle.AngleType.NEG_180_TO_180_HEADING),.9, this);
    }
}