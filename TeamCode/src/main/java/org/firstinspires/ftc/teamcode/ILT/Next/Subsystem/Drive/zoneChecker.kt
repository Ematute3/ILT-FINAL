package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive

import com.pedropathing.geometry.Pose
import kotlin.math.abs

class ZoneChecker(
    // gotta change this
    private val robotWidth: Double = 13.0,
    private val robotLength: Double = 13.0
) {

    // Define field zones as triangles
    private val obstacle = listOf(Pose(0.0, 115.0), Pose(25.0, 144.0), Pose(0.0, 141.0))
    private val upper = listOf(Pose(0.0, 115.0), Pose(25.0, 144.0), Pose(72.0, 72.0))
    private val lower = listOf(Pose(48.0, 0.0), Pose(72.0, 24.0), Pose(72.0, 0.0))

    // Check if point p is inside triangle defined by vertices a, b, c using barycentric coordinates
    private fun poseInTriangle(p: Pose, a: Pose, b: Pose, c: Pose): Boolean {
        val det = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y)
        if (abs(det) < 1e-6) return false
        val u = ((b.y - c.y) * (p.x - c.x) + (c.x - b.x) * (p.y - c.y)) / det
        val v = ((c.y - a.y) * (p.x - c.x) + (a.x - c.x) * (p.y - c.y)) / det
        val w = 1 - u - v
        return u >= 0 && v >= 0 && w >= 0
    }

    // Get robot bounding box corners
    private fun getRobotCorners(currentX: Double, currentY: Double): List<Pose> {
        val hw = robotWidth / 2.0
        val hl = robotLength / 2.0
        return listOf(
            Pose(currentX - hw, currentY - hl),
            Pose(currentX + hw, currentY - hl),
            Pose(currentX + hw, currentY + hl),
            Pose(currentX - hw, currentY + hl)
        )
    }

    // Check if any corner overlaps with a triangle
    private fun overlaps(corners: List<Pose>, tri: List<Pose>): Boolean {
        return corners.any { poseInTriangle(it, tri[0], tri[1], tri[2]) }
    }

    // Check if robot is in a legal shooting zone
    // Returns true if robot's bounding box is in upper/lower safe zones and not in obstacle zone
    fun inShootZone(currentX: Double, currentY: Double): Boolean {
        val corners = getRobotCorners(currentX, currentY)

        // Valid if in safe zone and not in obstacle
        val inUpper = overlaps(corners, upper)
        val inLower = overlaps(corners, lower)
        val inObstacle = overlaps(corners, obstacle)

        return (inUpper || inLower) && !inObstacle
    }

    // Public function to check if a specific pose is in a triangle (if needed elsewhere)
    fun isPoseInTriangle(p: Pose, a: Pose, b: Pose, c: Pose): Boolean {
        return poseInTriangle(p, a, b, c)
    }
}