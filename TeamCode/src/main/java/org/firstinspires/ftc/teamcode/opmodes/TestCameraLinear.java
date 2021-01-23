package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drivecontrol.Robot;

public class TestCameraLinear extends LinearOpMode {

    Robot robot;
    public boolean willResetIMU = true;

    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true, false);
        robot.initIMU();
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        slamra.start();




        slamra.stop();

    }

}
