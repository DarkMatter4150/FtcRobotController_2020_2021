package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.util.AllianceColor
import org.firstinspires.ftc.teamcode.robot.util.TeleOpUtil
import org.firstinspires.ftc.teamcode.robot.util.TeleOpUtil.manipulate
import org.firstinspires.ftc.teamcode.robot.util.TeleOpUtil.updatePosition

@Config
@TeleOp(name = "Blue TeleOp")
class BlueTele : BaseOpMode() {

    private val timer = ElapsedTime()

    override fun setup() {
        manipulate(robot,gp1,gp2,timer, AllianceColor.BLUE)
        robot.deployer.up()
    }

    override fun runLoop() {
        updatePosition(robot,telemetry,timer,runtime)
        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.fieldOrientedDrive(robot, gp1)

            OpModeType.Autonomous -> {
                // Replace false here with a check to cancel the sequence
                if (false) robot.drivetrain.cancelSequence()
                if (!robot.drivetrain.isBusy) opModeType = OpModeType.TeleOp
            }
            else ->
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp
        }
    }
    companion object {
        var liftChangeSpeed: Int = 50
    }
}