package org.firstinspires.ftc.teamcode.robot.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

fun Pose2d.toVector(): Vector2d {
    return Vector2d(this.x, this.y)
}