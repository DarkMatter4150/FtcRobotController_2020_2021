package org.firstinspires.ftc.teamcode.opmodes.ultimategoal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Disabled
@TeleOp(name="Test T265 Full Op", group="Iterative Opmode")
public class PositionTracking extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;
    // We treat this like a singleton because there should only ever be one object per camera
    //T265Camera slamra = null;
    //private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //final int robotRadius = 9; // inches
    //TelemetryPacket packet = new TelemetryPacket();
    //Canvas field = packet.fieldOverlay();
   // Translation2d translationSLAM = new Translation2d();
   // Rotation2d rotationSLAM = new Rotation2d();

   //Pose2d startingPose = new Pose2d((0*0.0254), (0*0.0254), new Rotation2d());

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        //driveToLocation(0,25,0,1,1);


    }

    public void driveToLocation(double x, double y, double rotation, double speed, double tolerance) {
        //updateSLAMNav();
    /*    double currentX = translationSLAM.getX();
        double currentY = -translationSLAM.getY();
        double theta = -rotationSLAM.getDegrees();
        //TODO: Change currentX and currentY to getX() and getY()
        while (!(between(currentY,y-tolerance,y+tolerance)) || !(between(currentX,x-tolerance,x+tolerance))) {

            //robot.updateBulkData();
            //updateSLAMNav();

            double goToAngle = Math.atan2((y-currentY),(x-currentX));
            Vector2d goToVector = new Vector2d(goToAngle, goToAngle);
            goToVector = goToVector.scale(Math.sqrt(speed));

            robot.driveController.updateUsingJoysticks(goToVector, Vector2d.ZERO, false);

            telemetry.addData("X", getX());
            telemetry.addData("Y", getY());
            telemetry.addData("Rot", getRotation());
        }
        robot.driveController.update(Vector2d.ZERO, 0);*/



    }

    public static boolean between(double i, double minValueInclusive, double maxValueInclusive) {
        if (i >= minValueInclusive && i <= maxValueInclusive)
            return true;
        else
            return false;
    }

    /*public void updateSLAMNav() {
        // We divide by 0.0254 to convert meters to inches
        T265Camera slamra = new T265Camera(new Transform2d(), 0, hardwareMap.appContext);
        T265Camera.CameraUpdate up;
        slamra.start();
        up = slamra.getLastReceivedCameraUpdate();
        translationSLAM = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotationSLAM = up.pose.getRotation();

        field.strokeCircle(translationSLAM.getX(), translationSLAM.getY(), robotRadius);
        double arrowX = rotationSLAM.getCos() * robotRadius, arrowY = rotationSLAM.getSin() * robotRadius;
        double x1 = translationSLAM.getX() + arrowX / 2, y1 = translationSLAM.getY() + arrowY / 2;
        double x2 = translationSLAM.getX() + arrowX, y2 = translationSLAM.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
        slamra.stop();
    }

    public Rotation2d getRotation() {
        return rotationSLAM;
    }

    public double getX() {
        return translationSLAM.getX();
    }

    public double getY() {
        return -translationSLAM.getY();
    }*/

}