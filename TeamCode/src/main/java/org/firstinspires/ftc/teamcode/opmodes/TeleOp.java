package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.drivecontrol.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import org.firstinspires.ftc.teamcode.drivecontrol.Angle;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    public boolean willResetIMU = true;

    double imuTarget = 0;

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    private boolean slow = false;
    private boolean claw = false;
    private boolean pusherThing = true;
    private boolean boxLifter = true;
    private double boxTimer = 0;
    private double pusherTimer = 0;

    private double duckTimer = 0;
    private ElapsedTime timer = new ElapsedTime();

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

    //Gamepad 1
    float leftStick1x;
    float leftStick1y;
    float rightStick1x;
    float rightStick1y;
    float leftTrigger1;
    float rightTrigger1;
    boolean dpadLeft1;
    boolean dpadRight1;

    boolean dpadDown2;

    //Gamepad 2
    float leftStick2x;
    float leftStick2y;
    float rightStick2x;
    float rightStick2y;
    float leftTrigger2;
    float rightTrigger2;
    boolean aButton;
    boolean bButton;
    boolean xButton;
    boolean yButton;

    double lastTime;

    Vector2d joystick1, joystick2;

    public void init() {
        robot = new Robot(this, false, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
        lastTime = getRuntime();
    }


    public void start () {
        if (willResetIMU) robot.initIMU();
        imuTarget = robot.getRobotHeading().getAngle();
    }

    public void loop() {

        loopStartTime = System.currentTimeMillis();
        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        leftStick2x = gamepad2.left_stick_x;
        leftStick2y = -gamepad2.left_stick_y;
        rightStick2x = gamepad2.right_stick_x;
        rightStick2y = -gamepad2.right_stick_y;
        leftTrigger2 = gamepad2.left_trigger;
        rightTrigger2 = gamepad2.right_trigger;
        aButton = gamepad2.a;
        bButton = gamepad2.b;
        xButton = gamepad2.x;
        yButton = gamepad2.y;
        dpadLeft1 = gamepad1.dpad_left;
        dpadRight1 = gamepad1.dpad_right;


        leftStick1x = gamepad1.left_stick_x;
        leftStick1y = gamepad1.left_stick_y;
        rightStick1x = gamepad1.right_stick_x;
        rightStick1y = gamepad1.right_stick_y;
        leftTrigger1 = gamepad1.left_trigger;
        rightTrigger1 = gamepad1.right_trigger;
        dpadDown2 = gamepad2.dpad_down;

        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        telemetry.addData("Robot Heading: ", robot.getRobotHeading());
        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));
        telemetry.addData("Left Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
        telemetry.addData("Right Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());

        telemetry.update();


        //Swerve Driving Located in this method
        driveFunctions();
        manipulators();

    }

    public void stop() {
        robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
        super.stop();
    }

    public void driveFunctions() {
        joystick1 = new Vector2d(leftStick1x, -leftStick1y); //LEFT joystick
        joystick2 = new Vector2d(rightStick1x+((rightStick1x/Math.abs(rightStick1x))*0.1), -rightStick1y); //RIGHT joystick
        slowModeDrive = false;

        //slow mode/range stuffs
        if (leftTrigger1 > 0.1) {
            // joystick1 = joystick1.scale(0.3);
            // joystick2 = joystick2.scale(0.4); //was 0.3
            joystick1 = joystick1.scale((1-Math.abs(leftTrigger1))*.75);
            joystick2 = joystick2.scale(1-Math.abs(leftTrigger1));
            slowModeDrive = true;
        }
        robot.driveController.updateUsingJoysticks(
                checkDeadband(joystick1, slowModeDrive).scale(Math.sqrt(2)),
                checkDeadband(joystick2, slowModeDrive).scale(Math.sqrt(2)),
                absHeadingMode
        );

        if (dpadLeft1) {
            robot.driveController.setDrivingStyle(true);
        } else if (dpadRight1) {
            robot.driveController.setDrivingStyle(false);
        }
    }

    public void manipulators() {
        //Manipulators
        robot.setIntakePower(leftTrigger2);
        robot.setDuckSpinnerPower((float) (-rightTrigger2*.8));

        if (xButton) {
            robot.setDuckSpinnerPower(-1);
        }
        else {
            robot.setDuckSpinnerPower((float) (-rightTrigger2*.78));
        }

        //Intake Servo
        if (bButton && timer.milliseconds() - boxTimer > 250) {
            boxLifter = !boxLifter;
            robot.setIntakeServo(boxLifter);
            boxTimer = timer.milliseconds();
        }

        //Duck Spinner Automatic
        if (yButton) {
            duckTimer = timer.milliseconds();
            while (timer.milliseconds() - duckTimer < 1100 && !dpadDown2) {
                float speed = (float) (.0009*(timer.milliseconds() - duckTimer)+.1);
                robot.setDuckSpinnerPower(-speed);
            }
            while (timer.milliseconds() - duckTimer < 1400 && timer.milliseconds() - duckTimer >= 1100 && !dpadDown2) {
                robot.setDuckSpinnerPower(-1);
            }
            while (timer.milliseconds() - duckTimer < 1450 && timer.milliseconds() - duckTimer >= 1400 && !dpadDown2) {
                robot.setDuckSpinnerPower((float)0.1);
            }
        }
    }
    public Vector2d checkDeadband(Vector2d joystick, boolean slowMode) {
        if (joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL)) {
        }
        else {
        }
        return  joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL) ?
                joystick : new Vector2d(0, 0);
    }
}