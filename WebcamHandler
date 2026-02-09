package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.Collections;
import java.util.List;
import android.util.Size;
import org.firstinspires.ftc.vision.VisionPortal;

public class WebcamHandler {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Constructor: รับค่า HardwareMap และ Telemetry มาจาก class หลัก
    public WebcamHandler(HardwareMap hwMap, Telemetry tel) {
        this.hardwareMap = hwMap;
        this.telemetry = tel;
    }

    // ฟังก์ชันเริ่มต้นทำงาน (เรียกใช้ใน init)
    public void init() {
        // สร้างตัวประมวลผล AprilTag (เหมือนเดิม)
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // --- ส่วนที่แก้ใหม่ (ใช้ Builder) ---
        // เราจะกำหนดความละเอียด 640x480 (พอดีกับจอและเร็ว)
        // และใช้ MJPEG ซึ่งส่งภาพผ่าน WiFi ได้ลื่นกว่า
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480)) // ลดขนาดภาพเพื่อความลื่น
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // ใช้ MJPEG สตรีมไวขึ้น
                .enableLiveView(true) // เปิดให้แสดงภาพ (สำคัญสำหรับการดูบน Driver Hub)
                .setAutoStopLiveView(true) // ให้หยุดภาพอัตโนมัติเมื่อไม่ได้ใช้เพื่อประหยัดแบต
                .build();
    }

    // ฟังก์ชันสำหรับอ่านค่าและแสดงผล (เรียกใช้ใน while loop)
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // วนลูปเช็คทุก Tag ที่เจอ
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s ====", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown ====", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\n");
    }

    /**
     * ฟังก์ชันสำหรับให้ class อื่นดึงข้อมูล AprilTag ที่ตรวจพบล่าสุด
     * @return List ของ AprilTagDetection
     */
    public List<AprilTagDetection> getLatestDetections() {
        if (aprilTag != null) {
            return aprilTag.getDetections();
        }
        return Collections.emptyList();
    }
    
    public AprilTagDetection getBestDetection() {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    
    if (currentDetections.isEmpty()) {
        return null; // ไม่เห็น Tag
    }

    // *** Logic การเลือก Tag ***
    // ตัวอย่างที่ 1: เลือก Tag ที่เห็นตัวแรกสุด
    // return currentDetections.get(0); 

    // ตัวอย่างที่ 2: เลือก Tag ที่มี ID
    for (AprilTagDetection detection : currentDetections) {
        if (detection.id == 24) { 
            return detection;
        }
    }
    
    // ถ้าไม่เจอ Tag ID ที่ต้องการ ให้ส่งค่า null
    return null;
}

    // ฟังก์ชันปิดกล้อง (เรียกเมื่อจบโปรแกรม ถ้าจำเป็น)
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
