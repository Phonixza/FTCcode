package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range; // [NEW] นำเข้า Range

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // [NEW] นำเข้า AprilTag
import java.util.List; // [NEW] นำเข้า List

@TeleOp(name = "teleopvi")
public class teleopvi extends LinearOpMode {

    // === Hardware Declaration ===
    private DcMotor motor_FL, motor_BL, motor_FR, motor_BR, motor_Intake;
    private DcMotorEx motor_ShootingLeft, motor_ShootingRight;
    private Servo servo_angle;
    private Servo servo_shooting;   
    private Servo servo_conveyer;

    // === Constants ===
    private static final double TARGET_VELOCITY = 1100; 
    private static final double SERVO_STEP = 0.02;      
    private static final long DEBOUNCE_TIME = 10;      

    // === [NEW] Vision Control Constants (ค่าคงที่สำหรับระบบเล็ง) ===
    final double DESIRED_DISTANCE = 85.0; // ระยะห่างที่อยากให้จอด (นิ้ว)
    final int TARGET_TAG_ID = 24;          // ID ป้ายที่จะเล็ง (แก้เลขนี้ตามป้ายในสนาม)
    
    // ค่าความไว (Gain) - ปรับถ้าหุ่นส่ายหรือช้าไป
    final double DRIVE_GAIN  = 0.06;   // เดินหน้า/ถอย
    final double STRAFE_GAIN = 0.045;  // สไลด์ข้าง
    final double TURN_GAIN   = 0.045;   // หมุน
    final double MAX_AUTO_SPEED = 0.6; // จำกัดความเร็วสูงสุดตอนเล็ง

    // === Servo Positions ===
    private double servoAnglePosition = 0.0; 

    // Conveyer 3 Positions
    private final double[] CONVEYER_POSITIONS_DEG = {10.0, 75.0, 100.0};
    private int conveyerStep = 0; 

    // Shooting Servo Positions
    private static final double SHOOTING_REST_DEG = 150.0;    
    private static final double SHOOTING_FIRE_DEG = 250.0;    

    // === Button State Tracking ===
    private long lastAnglePress = 0;
    private long lastConveyerPress = 0;
    private boolean lastLeftBumperState = false;

    // === Vision ===
    private WebcamHandler webcam; 

    @Override
    public void runOpMode() {

        // === 1. Hardware Map ===
        motor_FL = hardwareMap.get(DcMotor.class, "motor2");
        motor_BL = hardwareMap.get(DcMotor.class, "motor0");
        motor_FR = hardwareMap.get(DcMotor.class, "motor3");
        motor_BR = hardwareMap.get(DcMotor.class, "motor1");

        motor_Intake = hardwareMap.get(DcMotor.class, "motor4");

        motor_ShootingLeft = hardwareMap.get(DcMotorEx.class, "motor5");
        motor_ShootingRight = hardwareMap.get(DcMotorEx.class, "motor6");

        servo_angle = hardwareMap.get(Servo.class, "servo0expand");
        servo_shooting = hardwareMap.get(Servo.class, "servo1expand");
        servo_conveyer = hardwareMap.get(Servo.class, "servo2expand");

        // === 2. Webcam Init ===
        webcam = new WebcamHandler(hardwareMap, telemetry);
        webcam.init();

        // === 3. Motor Direction & Setup ===
        motor_FL.setDirection(DcMotor.Direction.FORWARD);
        motor_BL.setDirection(DcMotor.Direction.FORWARD);
        motor_FR.setDirection(DcMotor.Direction.REVERSE);
        motor_BR.setDirection(DcMotor.Direction.REVERSE);

        motor_Intake.setDirection(DcMotor.Direction.REVERSE);

        motor_ShootingLeft.setDirection(DcMotor.Direction.REVERSE);
        motor_ShootingRight.setDirection(DcMotor.Direction.FORWARD);

        motor_ShootingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingLeft.setVelocityPIDFCoefficients(18, 0, 0, 25); 
        motor_ShootingRight.setVelocityPIDFCoefficients(18, 0, 0, 25);

        // === 4. Set Initial Positions ===
        servo_angle.setPosition(servoAnglePosition);
        servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
        servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));

        telemetry.addData("Status", "Robot Ready!");
        telemetry.addData("Auto-Aim", "Hold L-Bumper (G1) to Align");
        telemetry.update();

        waitForStart();

        // === Main Loop ===
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // ----------------------------------------
            // 1. DRIVE SYSTEM (Modified for Vision)
            // ----------------------------------------
            
            // [NEW] เช็คว่ากดปุ่มช่วยเล็ง (L-Bumper Gamepad 1) ค้างไว้หรือไม่?
            if (gamepad1.left_bumper) {
                // === โหมดอัตโนมัติ (Auto-Align) ===
                alignToAprilTag(TARGET_TAG_ID);
                
            } else {
                // === โหมดขับเอง (Manual Drive) ===
                double axial   = -gamepad1.left_stick_y;  
                double lateral = gamepad1.left_stick_x;   
                double yaw     = gamepad1.right_stick_x;  

                // รวมแรง (Manual)
                double fl = axial + lateral + yaw;
                double fr = axial - lateral - yaw;
                double bl = axial - lateral + yaw;
                double br = axial + lateral - yaw;

                // Normalize
                double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
                if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

                motor_FL.setPower(fl);
                motor_FR.setPower(fr);
                motor_BL.setPower(bl);
                motor_BR.setPower(br);
            }

            // ----------------------------------------
            // 2. Intake
            // ----------------------------------------
            if (gamepad2.a) {
                motor_Intake.setPower(1.0);   
            } else if (gamepad2.y) {
                motor_Intake.setPower(-1.0);  
            } else {
                motor_Intake.setPower(0.0);   
            }

            // ----------------------------------------
            // 3. ปรับมุมยิง (Gamepad 2 D-Pad Up/Down)
            // ----------------------------------------
            if (now - lastAnglePress > DEBOUNCE_TIME) {
                if (gamepad2.dpad_up) {
                    servoAnglePosition = Math.min(servoAnglePosition + SERVO_STEP, 1.0);
                    servo_angle.setPosition(servoAnglePosition);
                    lastAnglePress = now;
                }
                if (gamepad2.dpad_down) {
                    servoAnglePosition = Math.max(servoAnglePosition - SERVO_STEP, 0.0);
                    servo_angle.setPosition(servoAnglePosition);
                    lastAnglePress = now;
                }
            }

            // ----------------------------------------
            // 4. Shooter Motor (Gamepad 2 Right Bumper)
            // ----------------------------------------
            if (gamepad2.right_bumper) {
                motor_ShootingLeft.setVelocity(TARGET_VELOCITY);
                motor_ShootingRight.setVelocity(TARGET_VELOCITY);
            } else {
                motor_ShootingLeft.setPower(0);
                motor_ShootingRight.setPower(0);
            }

            // ----------------------------------------
            // 5. Burst Fire Mode (ยิงรัว 3 นัด) - Gamepad 1 Right Trigger
            // ----------------------------------------
            if (gamepad1.right_trigger > 0.1) {
                for (int i = 0; i < 3; i++) {
                    if (!opModeIsActive()) break;
                    
                    // A. ยิง (Fire)
                    servo_shooting.setPosition(degToServo270(SHOOTING_FIRE_DEG));
                    sleep(300); 

                    // B. ดึงกลับ (Retract)
                    servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
                    sleep(200); 
                    motor_Intake.setPower(1.0);   
                    sleep(100);

                    // C. ขยับ Conveyer
                    if (i < 2) {
                        conveyerStep++;
                        if (conveyerStep >= CONVEYER_POSITIONS_DEG.length) {
                            conveyerStep = 0;
                        }
                         motor_Intake.setPower(1.0);   
                         sleep(100);
                        double nextPos = CONVEYER_POSITIONS_DEG[conveyerStep];
                        servo_conveyer.setPosition(degToServo180(nextPos));
                        sleep(800); 
                    }
                }
                conveyerStep = 0;
                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
                sleep(200);
            }

            // ----------------------------------------
            // 6. Manual Single Fire
            // ----------------------------------------
            if (gamepad1.right_bumper) {
                servo_shooting.setPosition(degToServo270(SHOOTING_FIRE_DEG));
            } else {
                if (gamepad1.right_trigger <= 0.1) {
                    servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
                }
            }

            // ----------------------------------------
            // 7. Manual Conveyer Step
            // ----------------------------------------
            boolean currentLB = gamepad2.left_bumper;
            if (currentLB && !lastLeftBumperState && (now - lastConveyerPress > DEBOUNCE_TIME)) {
                conveyerStep = (conveyerStep + 1) % 3; 
                double targetDeg = CONVEYER_POSITIONS_DEG[conveyerStep];
                servo_conveyer.setPosition(degToServo180(targetDeg));
                lastConveyerPress = now;
            }
            lastLeftBumperState = currentLB;

            if (gamepad2.dpad_left) {
                conveyerStep = 2; 
                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[2]));
            }
            else if (gamepad2.dpad_right) {
                conveyerStep = 0; 
                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
            }

            // ----------------------------------------
            // 8. Telemetry
            // ----------------------------------------
            webcam.telemetryAprilTag(); // Update Vision

            double rpmL = motor_ShootingLeft.getVelocity() * 60 / 28; 
            double rpmR = motor_ShootingRight.getVelocity() * 60 / 28;
            
            telemetry.addData("Mode", gamepad1.left_bumper ? "** AUTO ALIGN **" : "Manual Drive");
            telemetry.addData("Shooter", "%.0f RPM", rpmL);
            telemetry.update();
        }

        webcam.stop();
    }

    // =======================================================
    // [NEW] ฟังก์ชันคำนวณการเล็ง (Vision Logic)
    // =======================================================
    private void alignToAprilTag(int tagID) {
        AprilTagDetection target = null;
        List<AprilTagDetection> currentDetections = webcam.getLatestDetections();
        
        // 1. หาป้ายที่ต้องการ
        if (currentDetections != null) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == tagID) {
                    target = detection;
                    break;
                }
            }
        }

        if (target != null) {
            // เจอเป้าหมาย: คำนวณ Error
            double rangeError   = target.ftcPose.range - DESIRED_DISTANCE;
            double headingError = target.ftcPose.bearing - 0.3; // ซ้าย-ขวา
            double yawError     = target.ftcPose.yaw;     // เอียง

            // คำนวณ Power (P-Control)
            // หมายเหตุ: ถ้าหุ่นหมุนกลับด้าน ให้แก้เครื่องหมาย +/- ตรงนี้
            double drive  = Range.clip(rangeError * DRIVE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * -TURN_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

            // แสดงสถานะบนจอมือถือ
            telemetry.addData("Auto-Align", "Target Found! Dist: %.1f", target.ftcPose.range);
            
            // สั่งมอเตอร์
            moveRobot(drive, strafe, turn);

        } else {
            // ไม่เจอป้าย: หยุดหุ่นยนต์ (หรือจะให้หมุนหาช้าๆ ก็ได้)
            telemetry.addData("Auto-Align", "Scanning... (Target Not Found)");
            moveRobot(0, 0, 0); 
        }
    }

    // ฟังก์ชันรวมแรงขับ (Kinematics) - ใช้ร่วมกันทั้ง Manual และ Auto
    private void moveRobot(double x, double y, double yaw) {
        double fl = x + y + yaw;
        double fr = x - y - yaw;
        double bl = x - y + yaw;
        double br = x + y - yaw;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        motor_FL.setPower(fl);
        motor_FR.setPower(fr);
        motor_BL.setPower(bl);
        motor_BR.setPower(br);
    }

    // === Helper Functions ===

    private double degToServo270(double degrees) {
        return degrees / 270.0;
    }

    private double degToServo180(double degrees) {
        return degrees / 180.0;
    }
}
