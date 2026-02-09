package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "AutonomousB")
public class AutonomousB extends LinearOpMode {

    // === Drive Hardware ===
    private DcMotor motor_FL, motor_BL, motor_FR, motor_BR;

    // === Shooter Hardware (from test.java) ===
    private DcMotor motor_Intake;
    private DcMotorEx motor_ShootingLeft, motor_ShootingRight;
    private Servo servo_angle, servo_shooting, servo_conveyer;

    // === Modules ===
    private WebcamHandler webcam;
    private IMUHandler imu;

    // === Drive Constants ===
    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     DRIVE_GEAR_REDUCTION    = 3.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.8;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415 * 9);
    static final double     DRIVE_SPEED             = 0.3;

    // === IMU Constants ===
    static final double     P_TURN_GAIN             = 0.045;
    static final double     P_TURN_GAIN2             = 0.008;

    static final double     HEADING_THRESHOLD       = 3.5;

    // === AprilTag Constants ===
    private static final int    TARGET_TAG_ID           = 24;
    private static final double DESIRED_DISTANCE_INCH   = 42.0;
    static final double     STRAFE_GAIN             = 0.03;
    static final double     VISION_TURN_GAIN        = 0.03;
    static final double     MAX_AUTO_SPEED          = 0.5;
    // static final double     X_THRESHOLD_INCH        = 1.0;
    // static final double     YAW_THRESHOLD_DEG       = 2.0;

    // === Shooter Constants (from test.java) ===
    private static final double TARGET_VELOCITY = 2200;
    private final double[] CONVEYER_POSITIONS_DEG = {15, 60.0, 105.0};
    private static final double SHOOTING_REST_DEG = 150.0;
    private static final double SHOOTING_FIRE_DEG = 250.0;

    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {

        // === Hardware Map (Drive) ===
        motor_FL = hardwareMap.get(DcMotor.class, "motor2");
        motor_BL = hardwareMap.get(DcMotor.class, "motor0");
        motor_FR = hardwareMap.get(DcMotor.class, "motor3");
        motor_BR = hardwareMap.get(DcMotor.class, "motor1");

        // === Hardware Map (Shooter) ===
        motor_Intake = hardwareMap.get(DcMotor.class, "motor4");
        motor_ShootingLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        motor_ShootingRight = hardwareMap.get(DcMotorEx.class, "motor6");
        servo_angle = hardwareMap.get(Servo.class, "servo0expand");
        servo_shooting = hardwareMap.get(Servo.class, "servo1expand");
        servo_conveyer = hardwareMap.get(Servo.class, "servo2expand");

        // === Setup Modules ===
        webcam = new WebcamHandler(hardwareMap, telemetry);
        webcam.init();
        imu = new IMUHandler(hardwareMap, telemetry);
        imu.init();

        // === Drive Motor Setup ===
        motor_FL.setDirection(DcMotor.Direction.FORWARD);
        motor_BL.setDirection(DcMotor.Direction.FORWARD);
        motor_FR.setDirection(DcMotor.Direction.REVERSE);
        motor_BR.setDirection(DcMotor.Direction.REVERSE);
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        // === Shooter Motor Setup ===
        motor_Intake.setDirection(DcMotor.Direction.REVERSE);
        motor_ShootingLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_ShootingRight.setDirection(DcMotor.Direction.FORWARD);
        motor_ShootingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingLeft.setVelocityPIDFCoefficients(18, 0, 0, 25);
        motor_ShootingRight.setVelocityPIDFCoefficients(18, 0, 0, 25);

        // === Initial Servo Positions ===
        servo_angle.setPosition(0.66); // ตั้งค่ามุมเริ่มต้นสำหรับ Auto
        servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
        servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));


        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            blue();
        }

        webcam.stop();
    }

    // =================================================================
    // SHOOTING FUNCTION
    // =================================================================
    public void shoot(int Lo) {
        telemetry.addLine("Starting shooting sequence...");
        telemetry.update();

        servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[Lo]));
        sleep(150);
        // 1. Spin up shooter motors
        motor_ShootingLeft.setVelocity(TARGET_VELOCITY);
        motor_ShootingRight.setVelocity(TARGET_VELOCITY);
        telemetry.addLine("Spinning up motors...");
        telemetry.update();
        sleep(1600); // รอให้มอเตอร์ได้ความเร็ว


        // 3. Fire the spring mechanism
        telemetry.addLine("FIRE!");
        telemetry.update();

        servo_shooting.setPosition(degToServo270(SHOOTING_FIRE_DEG));
        sleep(100); // รอให้ servo ยิงเสร็จ
        servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));



        // 4. Reset for next shot

    }

    // =================================================================
    // MOVEMENT FUNCTIONS
    // =================================================================

    public void gyroTurn(double targetAngle) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            double error = targetAngle - imu.getHeading();
            if (Math.abs(error) <= HEADING_THRESHOLD) break;
            double turnPower = Range.clip(error * P_TURN_GAIN, -0.5, 0.5);
            setMecanumPower(0, -turnPower, 0);
        }
        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
        public void gyroTurn2(double targetAngl) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            double error = targetAngl - imu.getHeading();
            if (Math.abs(error) <= HEADING_THRESHOLD) break;
            double turnPower = Range.clip(error * P_TURN_GAIN2, -0.25, 0.25);
            setMecanumPower(0, -turnPower, 0);
        }
        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void correctWithAprilTag(int tagID) {
        telemetry.addLine("Tag Found! Aligning...");
        telemetry.update();
        
        // ต้องกำหนดโหมดการวิ่งใหม่เพื่อให้ setMecanumPower ทำงานได้อิสระ
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            AprilTagDetection target = null;
            List<AprilTagDetection> currentDetections = webcam.getLatestDetections();

            // 1. หาป้ายที่ต้องการ (Logic ใหม่)
            if (currentDetections != null) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null && detection.id == tagID) {
                        target = detection;
                        break;
                    }
                }
            }

            if (target != null) {
                // เจอเป้าหมาย: คำนวณ Error (Logic ใหม่)
                double rangeError   = target.ftcPose.range - DESIRED_DISTANCE_INCH;
                double headingError = target.ftcPose.bearing - 0.2; // ซ้าย-ขวา (มี Offset ตามโค้ดใหม่)
                double yawError     = target.ftcPose.yaw;           // เอียง (ใช้ yaw แทน x ตามโค้ดใหม่)

                // ตรวจสอบว่าถึงจุดหมายหรือยัง (Threshold เดิม เพื่อให้ Auto จบงานได้)
                if (Math.abs(rangeError) < 2.5 && Math.abs(headingError) < 2.5 && Math.abs(yawError) < 2.5) {
                    break; 
                }

                // คำนวณ Power (P-Control)
                // DRIVE_GAIN ใช้ 0.02 ตามมาตรฐานเดิม
                double drive  = Range.clip(rangeError * 0.049, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                // สังเกต: โค้ดใหม่ใส่ลบที่ Gain (-TURN_GAIN) ผมจึงใส่ลบตามที่คุณขอ
                double turn   = Range.clip(headingError * -VISION_TURN_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                // แสดงสถานะบนจอมือถือ
                telemetry.addData("Auto-Align", "Target Found! Dist: %.1f", target.ftcPose.range);
                telemetry.addData("Errors", "Range:%.1f, Head:%.1f, Yaw:%.1f", rangeError, headingError, yawError);
                telemetry.update();

                // สั่งมอเตอร์ (Mapping จาก moveRobot -> setMecanumPower)
                // setMecanumPower รับค่า (axial/drive, yaw/turn, lateral/strafe)
                setMecanumPower(drive, turn, strafe);

            } else {
                // ไม่เจอป้าย: หยุดหุ่นยนต์
                telemetry.addData("Auto-Align", "Scanning... (Target Not Found)");
                telemetry.update();
                sleep(300);
                // gyroTurn2(20);
                // sleep(300);
                // gyroTurn2(-20);
                // sleep(300);
                
            }
        }
        
        // จบการทำงาน หยุดมอเตอร์และคืนค่าโหมด
        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        // ตรวจสอบว่า OpMode ยังทำงานอยู่
        if (!opModeIsActive()) return;

        int moveLeftCounts  = (int)(leftInches * COUNTS_PER_INCH);
        int moveRightCounts = (int)(rightInches * COUNTS_PER_INCH);

        // --- แก้ไข: ให้แต่ละมอเตอร์คำนวณเป้าหมายจากตำแหน่งปัจจุบันของตัวเอง ---
        int flTarget = motor_FL.getCurrentPosition() + moveLeftCounts;
        int blTarget = motor_BL.getCurrentPosition() + moveLeftCounts;
        int frTarget = motor_FR.getCurrentPosition() + moveRightCounts;
        int brTarget = motor_BR.getCurrentPosition() + moveRightCounts;

        motor_FL.setTargetPosition(flTarget);
        motor_BL.setTargetPosition(blTarget);
        motor_FR.setTargetPosition(frTarget);
        motor_BR.setTargetPosition(brTarget);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(Math.abs(speed));

        double startTime = getRuntime();
        // แก้ไข Loop: เช็คว่ามอเตอร์ทั้ง 4 ตัวยังทำงานอยู่หรือไม่ (เพื่อความปลอดภัยกว่า)
        while (opModeIsActive() && (getRuntime() - startTime < timeoutS) &&
              (motor_FL.isBusy() && motor_FR.isBusy() && motor_BL.isBusy() && motor_BR.isBusy())) {
            
            // (Optional) ถ้าต้องการแก้เอียงด้วย IMU ต้องทำ P-Controller ตรงนี้ 
            // แต่สำหรับการวิ่ง Encoder ธรรมดา ปล่อยว่างไว้ได้
        }

        setMotorPower(0);
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // =================================================================
    // HELPER FUNCTIONS
    // =================================================================

    private boolean findAprilTag(int tagID) {
        desiredTag = null;
        List<AprilTagDetection> currentDetections = webcam.getLatestDetections();
        if (currentDetections == null) return false;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagID) {
                desiredTag = detection;
                return true;
            }
        }
        return false;
    }

    private double degToServo270(double degrees) {
        return degrees / 270.0;
    }

    private double degToServo180(double degrees) {
        return degrees / 180.0;
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        motor_FL.setMode(mode); motor_BL.setMode(mode);
        motor_FR.setMode(mode); motor_BR.setMode(mode);
    }

    private void setMotorPower(double power) {
        motor_FL.setPower(power); motor_BL.setPower(power);
        motor_FR.setPower(power); motor_BR.setPower(power);
    }
    
    private void setMotorZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        motor_FL.setZeroPowerBehavior(behavior);
        motor_BL.setZeroPowerBehavior(behavior);
        motor_FR.setZeroPowerBehavior(behavior);
        motor_BR.setZeroPowerBehavior(behavior);
    }

    public void setMecanumPower(double axial, double yaw, double lateral) {
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        motor_FL.setPower(fl); motor_FR.setPower(fr);
        motor_BL.setPower(bl); motor_BR.setPower(br);
    }
    
    public void blue() {
    
            // // --- Step 1: วิ่งออกจากจุด Start (Encoder) ---
            // encoderDrive(DRIVE_SPEED, 68, 68, 10);
            // sleep(200);

            
            // // --- Step 2: เลี้ยวหาป้าย (IMU Gyro Turn) ---
            // gyroTurn(30);
            // sleep(200);
            
            
            // correctWithAprilTag(TARGET_TAG_ID);

            // // --- Step 3: แก้ไขตำแหน่งให้แม่นยำด้วย AprilTag ---
            // // correctWithAprilTag(TARGET_TAG_ID);

            // // --- Step 4: [NEW] ยิง ---
            servo_angle.setPosition(0.05);
            sleep(200);
            shoot(0);
            sleep(200);
            motor_Intake.setPower(0.32);
            sleep(200);
            shoot(1);
            sleep(200);
            shoot(2);
            sleep(600);



            servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
            motor_ShootingLeft.setPower(0);
            motor_ShootingRight.setPower(0); 
            sleep(200);
            encoderDrive(DRIVE_SPEED, 25, 25, 10);
            //encoderDrive(DRIVE_SPEED, -50, -50, 10);
            telemetry.addLine("Shooting sequence complete.");
            telemetry.update();
            sleep(200);
            telemetry.addLine("Mission Complete!");
            telemetry.update();
            sleep(2000);
        
    }
    public void red() {
        
            // --- Step 1: วิ่งออกจากจุด Start (Encoder) ---
            encoderDrive(DRIVE_SPEED, 68, 68, 10);
            // sleep(200);

            
            // // --- Step 2: เลี้ยวหาป้าย (IMU Gyro Turn) ---
            // gyroTurn(30);
            // sleep(200);
            
            
            // correctWithAprilTag(TARGET_TAG_ID);

            // // --- Step 3: แก้ไขตำแหน่งให้แม่นยำด้วย AprilTag ---
            // // correctWithAprilTag(TARGET_TAG_ID);

            // // --- Step 4: [NEW] ยิง ---
            // shoot(0);
            // sleep(100);
            // motor_Intake.setPower(0.32);
            // sleep(100);
            // shoot(1);
            // sleep(100);
            // shoot(2);
            // sleep(100);



            // servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
            // motor_ShootingLeft.setPower(0);
            // motor_ShootingRight.setPower(0); 
            // sleep(200);
            // encoderDrive(DRIVE_SPEED, -50, -50, 10);
            // telemetry.addLine("Shooting sequence complete.");
            // telemetry.update();
            // sleep(500);
            // telemetry.addLine("Mission Complete!");
            // telemetry.update();
            // sleep(2000);
        
    }
}


