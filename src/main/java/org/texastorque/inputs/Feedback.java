package org.texastorque.inputs;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.texastorque.torquelib.base.TorqueFeedback;
import org.texastorque.torquelib.util.RollingMedian;

public class Feedback {
    private static volatile Feedback instance;

    public enum GyroDirection {
        CLOCKWISE, COUNTERCLOCKWISE;
    }

    private GyroFeedback gyroFeedback;
    private LimelightFeedback limelightFeedback;
    private ShooterFeedback shooterFeedback;

    private Feedback() {
        gyroFeedback = new GyroFeedback();
        limelightFeedback = new LimelightFeedback();
        shooterFeedback = new ShooterFeedback();
    }

    public void update() {
        gyroFeedback.update();
        limelightFeedback.update();
        shooterFeedback.update();
    }

    public void smartDashboard() {
        gyroFeedback.smartDashboard();
        limelightFeedback.smartDashboard();
        shooterFeedback.smartDashboard();
    }

    public class GyroFeedback extends TorqueFeedback {
        private final AHRS nxGyro;

        private double pitch;
        private double yaw;
        private double roll;

        private GyroDirection direction = GyroDirection.CLOCKWISE;

        private GyroFeedback() {
            nxGyro = new AHRS(SPI.Port.kMXP);
            nxGyro.getFusedHeading();
        }

        @Override
        public void update() {
            pitch = nxGyro.getPitch();
            roll = nxGyro.getRoll();

            double yaw_t = getDegrees();
            if (yaw_t - yaw > 0.3) {
                direction = GyroDirection.CLOCKWISE;
            } else {
                direction = GyroDirection.COUNTERCLOCKWISE;
            }
            yaw = yaw_t;
        }

        public void resetGyro() {
            nxGyro.reset();
        }

        public void zeroYaw() {
            nxGyro.zeroYaw();
        }

        public Rotation2d getRotation2d() {
            return Rotation2d.fromDegrees(getDegrees());
        }

        public Rotation2d getCCWRotation2d() {
            return Rotation2d.fromDegrees(getCCWDegrees());
        }

        public GyroDirection getGyroDirection() {
            return direction;
        }

        private float getDegrees() {
            // return nxGyro.getRoll();
            return nxGyro.getFusedHeading();
        }

        private float getCCWDegrees() {
            return 360.0f - nxGyro.getFusedHeading();
        }

        @Override
        public void smartDashboard() {
            SmartDashboard.putNumber("[FB]Gyro Pitch", pitch);
            SmartDashboard.putNumber("[FB]Gyro Yaw", yaw);
            SmartDashboard.putNumber("[FB]Gyro Roll", roll);
            SmartDashboard.putNumber("[FB]Gyro Deg", getDegrees());
            SmartDashboard.putNumber("[FB]Gyro CCW Deg", getCCWDegrees());
        }
    }

    public class LimelightFeedback extends TorqueFeedback {
        private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        private NetworkTableEntry tx = limelightTable.getEntry("tx");
        private NetworkTableEntry ty = limelightTable.getEntry("ty");
        private NetworkTableEntry ta = limelightTable.getEntry("ta");

        private double hOffset;
        private double vOffset;
        private double taOffset;

        private RollingMedian distanceMedian;
        private double distance;

        public LimelightFeedback() {
            distanceMedian = new RollingMedian(5);
        }

        @Override
        public void update() {
            hOffset = tx.getDouble(0);
            vOffset = ty.getDouble(0);
            taOffset = ta.getDouble(0);
            distance = distanceMedian.calculate(hOffset);
        }

        /**
         * @return the hOffset
         */
        public double gethOffset() {
            return hOffset;
        }

        /**
         * @return the vOffset
         */
        public double getvOffset() {
            return vOffset;
        }

        /**
         * @return the taOffset
         */
        public double getTaOffset() {
            return taOffset;
        }

        /**
         * @return Median calculated distance to target
         */
        public double getDistance() {
            return distance;
        }

        public void smartDashboard() {
            SmartDashboard.putNumber("hOffset", hOffset);
            SmartDashboard.putNumber("vOffset", vOffset);
            SmartDashboard.putNumber("taOffset", taOffset);
            SmartDashboard.putNumber("distance", distance);
        }

    }

    public class ShooterFeedback extends TorqueFeedback {

        private double RPM;
        private double hoodPosition;

        @Override
        public void update() {
        }

        /**
         * @return the hoodPosition
         */
        public double getHoodPosition() {
            return hoodPosition;
        }

        /**
         * @return the RPM
         */
        public double getRPM() {
            return RPM;
        }

        /**
         * @param hoodPosition the hoodPosition to set
         */
        public void setHoodPosition(double hoodPosition) {
            this.hoodPosition = hoodPosition;
        }

        /**
         * @param RPM the RPM to set
         */
        public void setRPM(double RPM) {
            this.RPM = RPM;
        }

        public void smartDashboard() {
            SmartDashboard.putNumber("ShooterRPM", RPM);
            SmartDashboard.putNumber("HoodPosition", hoodPosition);
        }
    }

    public GyroFeedback getGyroFeedback() {
        return gyroFeedback;
    }

    public LimelightFeedback getLimelightFeedback() {
        return limelightFeedback;
    }

    public ShooterFeedback getShooterFeedback() {
        return shooterFeedback;
    }

    public static synchronized Feedback getInstance() {
        return (instance == null) ? instance = new Feedback() : instance;
    }
}