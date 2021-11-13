package org.firstinspires.ftc.teamcode.opmodes.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.PIDController;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Test", group = "TeleOp")
public class Test extends OpMode {
    Robot robot;
    public final double DEADBAND_MAG_NORMAL = 0.1;
    public final double DEADBAND_MAG_SLOW_MODE = 0.03;
    boolean slowModeDrive;
    public boolean willResetIMU = true;

    double imuTarget = 0;
    double error = 0;
    double lastError = 0;
    double errorSum = 0;
    double errorChange = 0;
    boolean usePIDforMovement = false;

    PIDController pidDrive = new PIDController(5, 2, 1);

    boolean absHeadingMode = false;

    double loopStartTime = 0;
    double loopEndTime = 0;

    private boolean slow = false;
    private boolean claw = false;
    private boolean pusherThing = true;
    private boolean boxLifter = true;
    private double boxTimer = 0;
    private double pusherTimer = 0;
    private ElapsedTime timer = new ElapsedTime();

    //hungry hippo (1 servo, 2 positions)
    //foundation grabber - latch (2 servos, 2 positions)
    //lift (2 motors, continuous)
    //intake (2 motors, continuous)
    //arm (2 servos, continuous)
    //grabber (1 servo, 2 positions)

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

        pidDrive.setSetpoint(imuTarget);
        pidDrive.setOutputRange(-1, 1);
        pidDrive.setInputRange(-180, 180);
        pidDrive.enable();
        imuTarget = robot.getRobotHeading().getAngle();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();

        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        joystick2 = new Vector2d(gamepad1.right_stick_x+((gamepad1.right_stick_x/Math.abs(gamepad1.right_stick_x))*0.1), -gamepad1.right_stick_y); //RIGHT joystick
        slowModeDrive = false;


        //Gamepad 2
        float leftStick2x = gamepad2.left_stick_x;
        float leftStick2y = -gamepad2.left_stick_y;
        float rightStick2x = gamepad2.right_stick_x;
        float rightStick2y = -gamepad2.right_stick_y;
        float leftTrigger2 = gamepad2.left_trigger;
        float rightTrigger2 = gamepad2.right_trigger;
        boolean aButton = gamepad2.a;
        boolean bButton = gamepad2.b;
        boolean xButton = gamepad2.x;
        boolean yButton = gamepad2.y;

        robot.setIntakePower(leftTrigger2);
        robot.setConveyorPower(-leftTrigger2);
        robot.setDuckSpinnerPower((float) (-rightTrigger2*.8));

        if (xButton) {
            robot.setDuckSpinnerPower(-1);
        }
        else {
            robot.setDuckSpinnerPower((float) (-rightTrigger2*.78));
        }

        if (yButton) {
            robot.setIntakePower(-1);
            robot.setConveyorPower(1);
        }


        if (aButton && timer.milliseconds() - pusherTimer > 250 && (boxLifter || pusherThing)) {
            pusherThing = !pusherThing;
            //robot.setPusherThing(pusherThing);
            pusherTimer = timer.milliseconds();
        }
        if (bButton && timer.milliseconds() - boxTimer > 250) {
            boxLifter = !boxLifter;
            //robot.setBoxLifter(boxLifter);
            boxTimer = timer.milliseconds();
        }


        telemetry.addData("Robot Heading: ", robot.getRobotHeading());

        telemetry.addData("Joystick 2 Angle (180 heading mode): ", joystick2.getAngleDouble(Angle.AngleType.NEG_180_TO_180_HEADING));
        telemetry.addData("Heading to joystick difference: ", joystick2.getAngle().getDifference(robot.getRobotHeading()));

        telemetry.addData("Left Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation());
        telemetry.addData("Right Orientation: ", robot.driveController.moduleRight.getCurrentOrientation());



        //slow mode/range stuffs
        if (gamepad1.left_trigger > 0.1) {
            // joystick1 = joystick1.scale(0.3);
            // joystick2 = joystick2.scale(0.4); //was 0.3
            joystick1 = joystick1.scale((1-Math.abs(gamepad1.left_trigger))*.75);
            joystick2 = joystick2.scale(1-Math.abs(gamepad1.left_trigger));
            slowModeDrive = true;
        }


        if (usePIDforMovement) {
            if (joystick1.getMagnitude() >= .1 && joystick2.getMagnitude() <= .1) {
                pidDrive.setSetpoint(imuTarget);
                //double pidTurn = pidController(imuTarget, robot.getRobotHeading().getAngle(), 2,1,1);
                double pidTurn = pidDrive.performPID(robot.getRobotHeading().getAngle());
                joystick2 = new Vector2d(joystick2.getX()-pidTurn, joystick2.getY());
            }

        }

        robot.driveController.updateUsingJoysticks(
                checkDeadband(joystick1, slowModeDrive).scale(Math.sqrt(2)),
                checkDeadband(joystick2, slowModeDrive).scale(Math.sqrt(2)),
                absHeadingMode
        );


        if (gamepad1.dpad_left) {
            robot.driveController.setDrivingStyle(true);
        } else if (gamepad1.dpad_right) {
            robot.driveController.setDrivingStyle(false);
        }

        //todo: remove after done tuning
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//
//        if (gamepad2.dpad_right) {
//            robot.LIFT_TICKS_PER_MS += 0.05;
//        } else if (gamepad2.dpad_left) {
//            robot.LIFT_TICKS_PER_MS -= 0.05;
//        }
//
//        if (gamepad1.right_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG += 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG += 0.01;
//        }
//
//        if (gamepad1.left_bumper) {
//            robot.driveController.moduleRight.ROBOT_ROTATION_MAX_MAG -= 0.01;
//            robot.driveController.moduleLeft.ROBOT_ROTATION_MAX_MAG -= 0.01;
//        }

        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


        telemetry.addData("joystick 1", joystick1);
        telemetry.addData("joystick 2", joystick2);

        loopEndTime = System.currentTimeMillis();
        telemetry.addData("Our loop time: ", loopEndTime - loopStartTime);
        telemetry.addData("imuTarget", imuTarget);

        telemetry.update();
    }

    public void stop() {
        robot.driveController.updateUsingJoysticks(new Vector2d(0, 0), new Vector2d(0, 0), false);
        pidDrive.disable();
        super.stop();
    }

    public Vector2d checkDeadband(Vector2d joystick, boolean slowMode) {
        if (joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL)) {
            imuTarget = robot.getRobotHeading().getAngle();
        }
        else {
        }
        return  joystick.getMagnitude() > (slowMode ? DEADBAND_MAG_SLOW_MODE : DEADBAND_MAG_NORMAL) ?
                joystick : new Vector2d(0, 0);
    }

    public double pidController(double target, double current, double Kp, double Ki, double Kd) {
        error = target-current;
        errorChange = error-lastError;
        errorSum += error;
        double correction = Kp*error + Ki*errorSum + Kd*errorChange;
        lastError = error;
        return correction;
    }
}