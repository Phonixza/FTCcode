package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DECODE_Final_Full")
public class test extends LinearOpMode {

    // === Hardware Declaration ===
    private DcMotor motor_FL, motor_BL, motor_FR, motor_BR, motor_Intake;
    private DcMotorEx motor_ShootingLeft, motor_ShootingRight;
    private Servo servo_angle;
    private Servo servo_shooting;   // REV Servo 270° -> ใช้แค่ช่วงพักถึงยิง
    private Servo servo_conveyer;

    // === Constants ===
    private static final double TARGET_VELOCITY = 1300; // ความเร็วมอเตอร์ยิ
    private static final double TARGET_VELOCITY_HIGH = 2200;
    private static final double TARGET_VELOCITY_OUT = 550;
    private static final double SERVO_STEP = 0.02;      // ความละเอียดการปรับมุม
    private static final long DEBOUNCE_TIME = 10;     // เวลาหน่วงปุ่มกด (ms)

    // === Servo Positions ===
    private double servoAnglePosition = 0.0; // ตำแหน่งเริ่มต้นของมุมยิง

    // Conveyer 3 ตำแหน่ง (องศา)
    private final double[] CONVEYER_POSITIONS_DEG = {15.0, 60.0, 105.0};
    private int conveyerStep = 0; // 0 = ลูกแรก, 1 = ลูกสอง, 2 = ลูกสาม

    // Shooting Servo Positions (REV 270°)
    private static final double SHOOTING_REST_DEG = 150.0;    // ตำแหน่งพัก (ดึงกลับ)
    private static final double SHOOTING_FIRE_DEG = 260.0;    // ตำแหน่งยิง (ดีดออก)

    // === Button State Tracking ===
    private long lastAnglePress = 0;
    private long lastConveyerPress = 0;
    private boolean lastLeftBumperState = false;

    // === Vision ===
    private WebcamHandler webcam; // *ต้องมีไฟล์ WebcamHandler.java ในโปรเจกต์*

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

        // ใช้ PID Control สำหรับล้อยิงเพื่อให้ความเร็วคงที่
        motor_ShootingLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_ShootingLeft.setVelocityPIDFCoefficients(18, 0, 0, 25); // จูนค่า PID ตามความเหมาะสม
        motor_ShootingRight.setVelocityPIDFCoefficients(18, 0, 0, 25);

        // === 4. Set Initial Positions ===
        servo_angle.setPosition(servoAnglePosition);
        servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
        servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));

        telemetry.addData("Status", "Robot Ready!");
        telemetry.addData("Controls", "G1: Drive, RT=BurstFire | G2: Shooter, Angle");
        telemetry.update();

        waitForStart();

        // === Main Loop ===
        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // ----------------------------------------
            // 1. Mecanum Drive (Gamepad 1)
            // ----------------------------------------
            double axial   = -gamepad1.left_stick_y;  // เดินหน้า-ถอยหลัง
            double lateral = gamepad1.left_stick_x;   // สไลด์ซ้าย-ขวา
            double yaw     = gamepad1.right_stick_x;  // หมุน

            double fl = axial + lateral + yaw;
            double fr = axial - lateral - yaw;
            double bl = axial - lateral + yaw;
            double br = axial + lateral - yaw;

            // Normalize values
            double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

            motor_FL.setPower(fl);
            motor_FR.setPower(fr);
            motor_BL.setPower(bl);
            motor_BR.setPower(br);

            // ----------------------------------------
            // 2. Intake
            // ----------------------------------------
            if (gamepad2.a) {
                motor_Intake.setPower(0.80);   // กด A = ดูดเข้า
            } else if (gamepad2.y) {
                motor_Intake.setPower(-0.80);  // กด Y = คายออก (หมุนกลับ)
            } else {
                motor_Intake.setPower(0.0);   // ไม่กด = หยุด
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
            } else if (gamepad2.b) {
                motor_ShootingLeft.setVelocity(-TARGET_VELOCITY_OUT);
                motor_ShootingRight.setVelocity(-TARGET_VELOCITY_OUT);
            } else {
                motor_ShootingLeft.setPower(0);
                motor_ShootingRight.setPower(0);
            }
            
            if (gamepad2.right_trigger > 0.1) {
                motor_ShootingLeft.setVelocity(TARGET_VELOCITY_HIGH);
                motor_ShootingRight.setVelocity(TARGET_VELOCITY_HIGH);
            } else if (gamepad2.b) {
                motor_ShootingLeft.setVelocity(-TARGET_VELOCITY_OUT);
                motor_ShootingRight.setVelocity(-TARGET_VELOCITY_OUT);
            } else {
                motor_ShootingLeft.setPower(0);
                motor_ShootingRight.setPower(0);
            }

            // ----------------------------------------
            // 5. Burst Fire Mode (ยิงรัว 3 นัด) - Gamepad 1 Right Trigger
            // ----------------------------------------
            if (gamepad1.right_trigger > 0.1) {
                // วนลูป 3 รอบเพื่อยิง 3 ลูก
                for (int i = 0; i < 3; i++) {
                    // Safety: เช็คเผื่อกด Stop กลางคัน
                    if (!opModeIsActive()) break;
                    
                    

                    // A. ยิง (Fire)
                    servo_shooting.setPosition(degToServo270(SHOOTING_FIRE_DEG));
                    sleep(150); // รอดีด

                    // B. ดึงกลับ (Retract)
                    servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
                    sleep(70); // รอกลับ
                     motor_Intake.setPower(0.80);   // กด A = ดูดเข้า
                     sleep(70);

                    // C. ขยับ Conveyer (ทำเฉพาะรอบที่ 1 และ 2)
                    // รอบสุดท้าย (i=2) ไม่ต้องขยับ เพราะเดี๋ยวจะรีเซ็ต
                    if (i < 2) {
                        conveyerStep++;
                        // ป้องกัน Array Index Out of Bounds
                        if (conveyerStep >= CONVEYER_POSITIONS_DEG.length) {
                            conveyerStep = 0;
                        }
                         motor_Intake.setPower(0.80);   // กด A = ดูดเข้า
                         sleep(50);
                        double nextPos = CONVEYER_POSITIONS_DEG[conveyerStep];
                        servo_conveyer.setPosition(degToServo180(nextPos));
                        sleep(500); // รอให้ลูกไหลลงมา
                    }
                }

                // D. รีเซ็ต Conveyer กลับตำแหน่งแรก (เตรียมรับลูกใหม่)
                conveyerStep = 0;


                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
                sleep(100);
            }

            // ----------------------------------------
            // 6. Manual Single Fire (ยิงทีละนัด) - Gamepad 1 Right Bumper
            // ----------------------------------------
            if (gamepad1.right_bumper) {
                servo_shooting.setPosition(degToServo270(SHOOTING_FIRE_DEG));
            } else {
                // ถ้าไม่ได้กด Burst Fire และไม่ได้กด Manual ให้กลับมาท่าพัก
                // (ต้องเช็ค logic ดีๆ เพื่อไม่ให้ตีกับ Burst Fire แต่เนื่องจาก Burst ใช้ sleep มันเลยไม่ตีกัน)
                if (gamepad1.right_trigger <= 0.1) {
                    servo_shooting.setPosition(degToServo270(SHOOTING_REST_DEG));
                }
            }

            // ----------------------------------------
            // 7. Manual Conveyer Step (Gamepad 2 Left Bumper)
            // ----------------------------------------
            boolean currentLB = gamepad2.left_bumper;
            if (currentLB && !lastLeftBumperState && (now - lastConveyerPress > DEBOUNCE_TIME)) {
                conveyerStep = (conveyerStep + 1) % 3; // วน 0 -> 1 -> 2 -> 0
                double targetDeg = CONVEYER_POSITIONS_DEG[conveyerStep];
                servo_conveyer.setPosition(degToServo180(targetDeg));
                lastConveyerPress = now;
            }
            lastLeftBumperState = currentLB;

            // B. ทางลัด D-Pad (Left = ไปตำแหน่ง 2, Right = ไปตำแหน่ง 0) [ส่วนที่เพิ่ม]
            if (gamepad2.dpad_left) {
                conveyerStep = 2; // อัปเดตตัวแปร Step เป็น 2
                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[2]));
            }
            else if (gamepad2.dpad_right) {
                conveyerStep = 0; // อัปเดตตัวแปร Step เป็น 0
                servo_conveyer.setPosition(degToServo180(CONVEYER_POSITIONS_DEG[0]));
            }


            // ----------------------------------------
            // 8. Telemetry & Vision Update
            // ----------------------------------------
            webcam.telemetryAprilTag(); // อัปเดตข้อมูลกล้อง

            double rpmL = motor_ShootingLeft.getVelocity() * 60 / 28; // คำนวณ RPM คร่าวๆ
            double rpmR = motor_ShootingRight.getVelocity() * 60 / 28;
            boolean isReady = rpmL >= 3800 && rpmR >= 3800; // เช็คว่ารอบถึงยัง (ตัวเลขสมมติ)

            telemetry.addData("--- Shooter Status ---", "");
            telemetry.addData("RPM", "L:%.0f  R:%.0f  [%s]", rpmL, rpmR, isReady ? "READY" : "WAIT");

            telemetry.addData("--- Mechanisms ---", "");
            telemetry.addData("Angle Servo", "%.3f (deg: %.1f)", servoAnglePosition, servoAnglePosition * 180);
            telemetry.addData("Conveyer Step", "%d (%.1f deg)", conveyerStep, CONVEYER_POSITIONS_DEG[conveyerStep]);
            telemetry.addData("Mode", gamepad1.right_trigger > 0.1 ? "BURST FIRE!!!" : "Manual");

            telemetry.update();
        }

        // จบการทำงาน
        webcam.stop();
    }
    
    

    // === Helper Functions ===

    // สำหรับ Servo 270 องศา (REV Smart Servo)
    // Input: องศา (0-270) -> Output: 0.0-1.0
    private double degToServo270(double degrees) {
        return degrees / 270.0;
    }

    // สำหรับ Servo 180 องศามาตรฐาน
    // Input: องศา (0-180) -> Output: 0.0-1.0
    private double degToServo180(double degrees) {
        return degrees / 180.0;
    }
}
