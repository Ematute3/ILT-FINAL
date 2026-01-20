package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.limeLight

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.next.subsystems.data.Motif

object limeLight : Subsystem {

    // FIX: Make nullable for safe initialization
    private var ll: Limelight3A? = null

    // FIX: Add initialization flag
    private var isInitialized = false

    // These are the values the Limelight gives us.
    var currentTx: Double = 0.0
        private set
    var currentTy: Double = 0.0
        private set
    var currentTa: Double = 0.0
        private set
    var hasValidTarget: Boolean = false
        private set

    var detectedMotif: Motif = Motif.NONE
        private set

    var fiducialCount: Int = 0
        private set
    var fiducialData: String = "No fiducials"
        private set

    override fun initialize() {
        try {
            ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "ll")
            ll?.let { camera ->
                camera.setPollRateHz(100)
                camera.pipelineSwitch(0)
                camera.start()
                isInitialized = true
                ActiveOpMode.telemetry.addData("Limelight", "Initialized Successfully")
            }
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Limelight Error", e.message)
            isInitialized = false
            ll = null
        }
    }

    override fun periodic() {
        // FIX: Only update if initialized
        if (!isInitialized || ll == null) {
            hasValidTarget = false
            return
        }

        updateBasicData()
        updateFiducialData()
        updateMotif()
    }

    private fun updateBasicData() {
        val result = ll?.latestResult
        if (result != null && result.isValid) {
            hasValidTarget = true
            currentTx = result.tx
            currentTy = result.ty
            currentTa = result.ta
        } else {
            hasValidTarget = false
            currentTx = 0.0
            currentTy = 0.0
            currentTa = 0.0
        }
    }

    private fun updateFiducialData() {
        val result = ll?.latestResult
        if (result != null && result.isValid) {
            val fiducials = result.fiducialResults
            fiducialCount = fiducials.size

            if (fiducials.isNotEmpty()) {
                val sb = StringBuilder()
                for (fr in fiducials) {
                    sb.append("ID: ${fr.fiducialId}, ")
                    sb.append("X: ${"%.2f".format(fr.targetXDegrees)}°, ")
                    sb.append("Strafe: ${"%.2f".format(fr.robotPoseTargetSpace.position.x)}\n")
                }
                fiducialData = sb.toString().trim()
            } else {
                fiducialData = "No fiducials detected"
            }
        } else {
            fiducialCount = 0
            fiducialData = "No valid result"
        }
    }

    private fun updateMotif() {
        val result = ll?.latestResult
        if (result != null && result.isValid) {
            val fR = result.fiducialResults
            if (fR.isNotEmpty()) {
                val f = fR[0]
                detectedMotif = when (f.fiducialId) {
                    21 -> Motif.GPP
                    22 -> Motif.PGP
                    else -> Motif.PPG
                }
            } else {
                detectedMotif = Motif.NONE
            }
        } else {
            detectedMotif = Motif.NONE
        }
    }

    fun grabResultData(): LLResult? {
        val lR = ll?.latestResult
        if (lR != null && lR.isValid) {
            return lR
        }
        return null
    }

    // FIX: Check pose validity before using heading
    fun megaTag(): Pose? {
        if (!isInitialized || ll == null) {
            return null
        }

        // FIX: Don't use pose if it's not valid
        if (!DriveTrain.isPoseValid()) {
            ActiveOpMode.telemetry.addData("MegaTag Warning", "DriveTrain pose not valid")
            return null
        }

        val lR = ll?.latestResult
        val yaw = DriveTrain.currentHeading

        try {
            ll?.updateRobotOrientation(yaw)
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("MegaTag Error", e.message)
            return null
        }

        if (lR != null && lR.isValid) {
            val botpose_mt2 = lR.botpose_MT2
            if (botpose_mt2 != null) {
                return Pose(botpose_mt2.position.x, botpose_mt2.position.y, yaw)
            }
        }
        return null
    }

    // FIX: Add method to check if Limelight is ready
    fun isReady(): Boolean {
        return isInitialized && ll != null
    }

    fun getTelemetryString(): String {
        return buildString {
            appendLine("=== LIMELIGHT STATUS ===")
            appendLine("Initialized: $isInitialized")
            appendLine("Valid Target: $hasValidTarget")
            if (hasValidTarget) {
                appendLine("TX: ${"%.2f".format(currentTx)}°")
                appendLine("TY: ${"%.2f".format(currentTy)}°")
                appendLine("TA: ${"%.2f".format(currentTa)}%")
            }
            appendLine("Fiducials: $fiducialCount")
            if (fiducialCount > 0) {
                appendLine(fiducialData)
            }
            appendLine("Motif: $detectedMotif")
        }
    }
}