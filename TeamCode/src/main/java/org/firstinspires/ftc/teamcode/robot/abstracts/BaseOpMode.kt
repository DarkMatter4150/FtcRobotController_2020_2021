/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.robot.abstracts

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

/**
 * Basic OpMode class that all OpModes should extend
 */
abstract class BaseOpMode : LinearOpMode() {
    protected lateinit var robot: CheckmateRobot
    protected lateinit var gp1: SuperController
    protected lateinit var gp2: SuperController

    protected enum class OpModeType {
        TeleOp, Autonomous
    }

    @JvmField protected var opModeType: OpModeType? = null

    /**
     * Runs before the hardware initializes
     */
    open fun preSetup() {}

    /**
     * Runs when the OpMode initializes
     */
    open fun setup() {}

    /**
     * Runs once before the loop starts
     */
    open fun preRunLoop() {}

    /**
     * Main OpMode loop, automatically updates the robot
     */
    open fun runLoop() {}

    /**
     * Runs when the OpMode is stopped
     */
    fun cleanup() {}

    final override fun runOpMode() {
        if (this.javaClass.getAnnotation(TeleOp::class.java) != null) {
            opModeType = OpModeType.TeleOp
        } else if (this.javaClass.getAnnotation(Autonomous::class.java) != null) {
            opModeType = OpModeType.Autonomous
        }
        preSetup()
        robot = CheckmateRobot(hardwareMap)
        gp1 = SuperController(gamepad1)
        gp2 = SuperController(gamepad2)
        setup()
        waitForStart()
        preRunLoop()
        while (opModeIsActive() && !isStopRequested) {
            if (opModeType == OpModeType.TeleOp) {
                gp1.update()
                gp2.update()
            }
            robot.update()
            telemetry.update()
            runLoop()
        }
        robot.cleanup()
        cleanup()
    }

}