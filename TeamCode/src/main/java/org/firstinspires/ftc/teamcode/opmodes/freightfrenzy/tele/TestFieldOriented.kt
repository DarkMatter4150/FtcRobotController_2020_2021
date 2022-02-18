package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables.TriggerableCallback

@Disabled
@TeleOp(name = "FieldOrientedTeleOp1")
class TestFieldOriented : BaseOpMode() {
    override fun setup() {
        gp1.y.onActivate = TriggerableCallback { robot.drivetrain.resetIMU() }
    }
    override fun runLoop() {
        robot.drivetrain.fieldOrientedDrive(robot, gp1)
    }
}