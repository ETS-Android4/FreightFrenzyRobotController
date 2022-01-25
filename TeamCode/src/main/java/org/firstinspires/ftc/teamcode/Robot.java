package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Robot {

    private HardwareMap hardwareMap;

    public DcMotorEx rbDrive, lbDrive, rfDrive, lfDrive;
    public DcMotor duckWheel;
    public DcMotor armMotorLeft, armMotorRight;
    public Servo armServoLeft, armServoRight;

    VuforiaLocalizer fieldLocalizer;

    // Servo Positions
    public static double[] CLAW_BACK = {0.05, 0};
    public static double[] CLAW_OPEN = {0.22, 0.3};
    public static double[] CLAW_GRAB = {0.45, 0.53};

    // Arm Positions
    public static int[] ARM_TOP = {1422, 1439};
    public static int[] ARM_MIDDLE = {1915, 1923};
    public static int[] ARM_BOTTOM = {2260, 2264};
    public static int[] ARM_BACK = {675, 685};

    public Robot(HardwareMap hw) {
        hardwareMap = hw;

        rbDrive  = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        lbDrive = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rfDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        lfDrive = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        duckWheel = hardwareMap.get(DcMotor.class, "wheel_motor");
        armMotorLeft = hardwareMap.get(DcMotor.class, "arm_motor_left");
        armMotorRight = hardwareMap.get(DcMotor.class, "arm_motor_right");
        armServoLeft = hardwareMap.get(Servo.class, "arm_servo_left");
        armServoRight = hardwareMap.get(Servo.class, "arm_servo_right");

        // Motor Directions
        rbDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        duckWheel.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Enable breaking on zero power
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run with encoders
        resetEncoders();
        useEncoders();

        // Servo Configuration
        armServoLeft.setDirection(Servo.Direction.REVERSE);
        armServoRight.setDirection(Servo.Direction.FORWARD);
    }

    public void resetEncoders() {
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders() {
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initializeVuforiaLocalizer() {
        fieldLocalizer = new VuforiaLocalizer(hardwareMap);
        fieldLocalizer.initialize();
    }

    public void drive(double drive, double strafe, double rotate, double speed) {
        final double rbPower = speed * (drive - rotate + strafe);
        final double lbPower = speed * (drive + rotate - strafe);
        final double rfPower = speed * (drive - rotate - strafe);
        final double lfPower = speed * (drive + rotate + strafe);

        rbDrive.setPower(Range.clip(rbPower, -1, 1));
        lbDrive.setPower(Range.clip(lbPower, -1, 1));
        rfDrive.setPower(Range.clip(rfPower, -1, 1));
        lfDrive.setPower(Range.clip(lfPower, -1, 1));
    }

    public void brake() {
        rbDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        lfDrive.setPower(0);
    }

    public void spinDuckWheel(double speed, int time) {
        duckWheel.setPower(speed);
        sleep(time);
        duckWheel.setPower(0);
    }

    public void spinDuckWheel(double speed) {
        duckWheel.setPower(speed);
    }

    public void setClawPosition(double[] positions) {
        armServoLeft.setPosition(positions[0]);
        armServoRight.setPosition(positions[1]);
    }

    public void setArmSpeed(double speed) {
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorLeft.setPower(speed);
        armMotorRight.setPower(speed);
    }

    public void setArmPosition(int[] position, double speed) {
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
        armMotorLeft.setTargetPosition(position[0]);
        armMotorRight.setTargetPosition(position[1]);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setPower(speed);
        armMotorRight.setPower(speed);
        new Thread(() -> {
            while (armIsBusy()) {}
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
            armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }).start();
    }

    public boolean armIsBusy() {
        return (armMotorLeft.isBusy() || armMotorLeft.isBusy());
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
