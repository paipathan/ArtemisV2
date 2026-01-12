package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimeLight {

    private static Limelight3A ll;
    private static IMU imu;

    public static void init(HardwareMap hwMap) {
        ll = hwMap.get(Limelight3A.class, "limelight");
        ll.pipelineSwitch(0);
        ll.setPollRateHz(100);
        ll.start();
        imu = hwMap.get(IMU.class, "imu");
    }

    public static boolean getRunning() {
        return ll.isRunning();
    }

    public static boolean getConnected() {
        return ll.isConnected();
    }

    public static boolean validResult() {
        return ll.getLatestResult().isValid();
    }

    public static double getTX() {
        return ll.getLatestResult().getTx();
    }

    public static double getTY() {
        return ll.getLatestResult().getTy();
    }

    public static double getTA() {
        return ll.getLatestResult().getTa();
    }

    public static double distanceFromTag(double tagID) {
        List<LLResultTypes.FiducialResult> r = ll.getLatestResult().getFiducialResults();

        if (r.isEmpty()) return 0;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult i: r) {
            if (i != null && i.getFiducialId() ==  tagID) {
                target = i;
                break;
            }
        }

        if (target != null) {
            double x = (target.getCameraPoseTargetSpace().getPosition().x / DistanceUnit.mPerInch);
            double z = (target.getCameraPoseTargetSpace().getPosition().z / DistanceUnit.mPerInch);

            Vector e = new Vector();
            e.setOrthogonalComponents(x, z);
            return e.getMagnitude();
        }

        return 0;
    }

    public static double distanceFromBlueGoal() {
        return distanceFromTag(20);
    }

    public static double distanceFromRedGoal() {
        return distanceFromTag(24);
    }

    public static LLResult getLatestResult() {
        if (ll == null) {
            return null;
        }
        return ll.getLatestResult();
    }

    public static Pose getRobotPose() {
        LLResult result = ll.getLatestResult();
        if (!result.isValid()) {
            return null;
        }

        Pose3D botPose = result.getBotpose();

        double xMeters = -botPose.getPosition().x;
        double yMeters = botPose.getPosition().y;
        double yawDegrees = botPose.getOrientation().getYaw();

        double xInches = (xMeters * 39.3701) + 72 ;
        double yInches = (yMeters * 39.3701) + 72;
        double headingRadians = Math.toRadians(yawDegrees - 90);

        return new Pose(
                yInches,
                xInches,
                headingRadians
        );
    }


}
