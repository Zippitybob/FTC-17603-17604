package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "LimelightTest")
@SuppressWarnings({"unused", "FieldCanBeLocal"})
public class LimelightTest extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Limelight3A limelight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        telemetry.log().add("Initializing hardware...");

        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse left motors so positive power drives all wheels forward
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.log().add("Motor and Limelight initialized successfully.");
        telemetry.update();

        waitForStart();
        telemetry.clear();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                // If running an AprilTag / Fiducial pipeline, pull values directly from the detected fiducial
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);
                    tx = tag.getTargetXDegrees();
                    ty = tag.getTargetYDegrees();
                    ta = tag.getTargetArea();
                    telemetry.addData("AprilTag ID", tag.getFiducialId());
                }

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // Driving and strafing code
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double scale = gamepad1.left_bumper ? 0.25 : 0.8;

            frontLeft.setPower(scale * (drive + strafe + rotate));
            backLeft.setPower(scale * (drive - strafe + rotate));
            frontRight.setPower(scale * (drive - strafe - rotate));
            backRight.setPower(scale * (drive + strafe - rotate));

            telemetry.update();
        }

        limelight.stop();
    }
}
