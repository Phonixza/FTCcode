package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IMUHandler {

    private IMU imu;
    private Telemetry telemetry;

    // *** ตั้งค่าทิศทางของ Control Hub ที่นี่ ***
    // ลองดูที่ตัวหุ่นว่าเอาด้าน Logo ขึ้นหรือลง และเอาช่อง USB หันไปทางไหน
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // Constructor
    public IMUHandler(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // เชื่อมต่อ Hardware
        imu = hardwareMap.get(IMU.class, "imu");
    }

    // ฟังก์ชันเริ่มทำงาน (เรียกใน init)
    public void init() {
        // สร้าง Parameters ตามทิศทางที่เรากำหนดข้างบน
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR));

        // สั่ง Initialize
        imu.initialize(parameters);

        // รีเซ็ตค่าให้เป็น 0 เริ่มต้น
        resetYaw();
    }

    // ฟังก์ชันอ่านค่ามุมหัน (Yaw/Heading)
    // คืนค่าเป็นองศา (+ คือซ้าย, - คือขวา)
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    // ฟังก์ชันรีเซ็ตมุมให้เป็น 0 (เช่น เมื่อกดปุ่ม หรือเริ่ม Auto)
    public void resetYaw() {
        imu.resetYaw();
    }

    // ฟังก์ชันเช็คว่า IMU ทำงานอยู่ไหม
    public void telemetryStatus() {
        telemetry.addData("IMU Heading", "%.2f Deg", getHeading());
    }
}
