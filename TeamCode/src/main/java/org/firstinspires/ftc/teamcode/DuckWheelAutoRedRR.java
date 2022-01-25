package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Duck Wheel Red",group = "drive")
@Config
public class DuckWheelAutoRedRR extends LinearOpMode {

    public static double ARM_SPEED = 0.45;
    public static double DUCKWHEEL_ROTATION_OFFSET = 5; // Degrees
    public static double DUCKWHEEL_SPIN_SPEED = -0.35;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        OpenCVElementTracker elementTracker = new OpenCVElementTracker(hardwareMap);
//        while (!elementTracker.streaming) {}
//        robot.initializeVuforiaLocalizer();

//        while (!elementTracker.streaming) {}

        Pose2d loc = new Pose2d(-32, -62, Math.toRadians(0));
        OpenCVElementTracker.LOCATION teamElementLoc;
        do {
//            loc = robot.fieldLocalizer.getLocationEstimate();
//            telemetry.addData("Location Estimate", "x: " + loc.getX() + ", y: " + loc.getY() + ", heading: " + loc.getHeading());
            teamElementLoc = elementTracker.getLocation();
        } while (!isStarted());

        runtime.reset();

        double hubDropoffOffset;
        int[] armPosition;
        switch (teamElementLoc) {
            case LEFT:
                armPosition = Robot.ARM_BOTTOM;
                hubDropoffOffset = 27.5;
                break;
            case RIGHT:
                armPosition = Robot.ARM_MIDDLE;
                hubDropoffOffset = 25;
                break;
            default:
                armPosition = Robot.ARM_TOP;
                hubDropoffOffset = 19.75;
        }

        drive.setPoseEstimate(loc);
        Trajectory toShippingHub = drive.trajectoryBuilder(loc)
                .lineTo(new Vector2d(-19, -24.75 - hubDropoffOffset))
                .build();
        Trajectory toDuckWheel = drive.trajectoryBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(-56, -60, Math.toRadians(-90)))
                .build();
        Trajectory toStorageUnit = drive.trajectoryBuilder(toDuckWheel.end()) // Math.toRadians(DUCKWHEEL_ROTATION_OFFSET)
                .lineTo(new Vector2d(-56, -43)) // spline with -90 tangent
                .build();

        robot.setArmPosition(armPosition, ARM_SPEED);
        while (robot.armIsBusy() && (runtime.seconds() < 3)) {
            if (isStopRequested()) return;
        }
//        telemetry.addData("Status", "arm finished");
        drive.followTrajectory(toShippingHub);
        robot.setClawPosition(Robot.CLAW_OPEN);
        sleep(1500);
        robot.setArmPosition(Robot.ARM_BACK, ARM_SPEED);
        drive.followTrajectory(toDuckWheel);
        drive.turn(Math.toRadians(DUCKWHEEL_ROTATION_OFFSET));
        robot.spinDuckWheel(DUCKWHEEL_SPIN_SPEED, 3000);
        drive.turn(Math.toRadians(-DUCKWHEEL_ROTATION_OFFSET));
        drive.followTrajectory(toStorageUnit);
    }

}
