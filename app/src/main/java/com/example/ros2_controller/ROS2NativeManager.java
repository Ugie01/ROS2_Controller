package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.wifi.WifiManager;
import android.util.Log;

import geometry_msgs.PoseStamped;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Twist;
import nav_msgs.OccupancyGrid;
import nav_msgs.Odometry;
import sensor_msgs.BatteryState;
import sensor_msgs.CompressedImage;
import sensor_msgs.LaserScan;
import std_msgs.String_;

import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

public class ROS2NativeManager {
    private static final String TAG = "ROS2_DEBUG_NativeMgr";

    public interface ROS2DataListener {
        void onCompressedImageReceived(Bitmap bitmap);
        void onLaserScanReceived(float[] ranges, float angleMin, float angleInc);
        void onMapReceived(byte[] data, int width, int height, float resolution, float originX, float originY);
        void onRobotPoseReceived(float worldX, float worldY, float yaw); // 로봇 위치 콜백
        void onBatteryStateReceived(float percentage, float voltage, boolean isCharging);
        void onCustomTopicReceived(String topic, String data);
        void onStatusChanged(String status, boolean isRunning);
        void onLog(String logMessage);
    }

    private final Context context;
    private final ROS2DataListener listener;
    private WifiManager.MulticastLock multicastLock;

    private ROS2Node ros2Node;
    private ROS2Publisher<Twist> cmdVelPublisher;
    private ROS2Publisher<PoseStamped> goalPublisher;
    private ROS2Publisher<String_> customStringPublisher;

    private ROS2Subscription<CompressedImage> imageSubscription;
    private ROS2Subscription<LaserScan> scanSubscription;
    private ROS2Subscription<OccupancyGrid> mapSubscription;
    private ROS2Subscription<PoseWithCovarianceStamped> amclPoseSubscription;
    private ROS2Subscription<Odometry> odomSubscription;
    private ROS2Subscription<BatteryState> batterySubscription;
    private ROS2Subscription<?> dynamicSubscription;

    private String currentSubTopic = "/chatter";
    private boolean isRunning = false;

    public ROS2NativeManager(Context context, ROS2DataListener listener) {
        this.context = context;
        this.listener = listener;
    }

    public synchronized void startNode(int domainId, boolean useBestEffortQos) {
        Log.i(TAG, "startNode 호출됨 - 도메인 ID: " + domainId);
        stopNode();

        try {
            Log.i(TAG, "[DDS] Wi-Fi Multicast Lock 획득 시도 중...");
            WifiManager wifi = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            if (wifi != null) {
                multicastLock = wifi.createMulticastLock("ros2_dds_multicast_lock");
                multicastLock.setReferenceCounted(true);
                multicastLock.acquire();
                log("MulticastLock 획득 완료");
            }

            log("ROS2Node 생성 중...");
            ros2Node = new ROS2Node("android_controller_node", domainId);
            log("ROS2Node 생성 완료: android_controller_node (Domain: " + domainId + ")");

            // 1. /cmd_vel 퍼블리셔
            ROS2Topic<Twist> cmdVelTopic = new ROS2Topic<>("/cmd_vel", Twist.class);
            cmdVelPublisher = ros2Node.createPublisher(cmdVelTopic);

            // 2. /goal_pose 퍼블리셔
            ROS2Topic<PoseStamped> goalTopic = new ROS2Topic<>("/goal_pose", PoseStamped.class);
            goalPublisher = ros2Node.createPublisher(goalTopic);

            // 3. /image_raw/compressed 구독자
            ROS2Topic<CompressedImage> imageTopic = new ROS2Topic<>("/image_raw/compressed", CompressedImage.class);
            imageSubscription = ros2Node.createSubscription(imageTopic, reader -> {
                try {
                    CompressedImage msg = reader.read();
                    if (msg != null && msg.getData() != null) {
                        byte[] data = msg.getData().getBuffer().array();
                        Bitmap bitmap = BitmapFactory.decodeByteArray(data, 0, data.length);
                        if (bitmap != null && listener != null) {
                            listener.onCompressedImageReceived(bitmap);
                        }
                    }
                } catch (Throwable ignored) {}
            });

            // 4. /scan LiDAR 구독자
            ROS2Topic<LaserScan> scanTopic = new ROS2Topic<>("/scan", LaserScan.class);
            scanSubscription = ros2Node.createSubscription(scanTopic, reader -> {
                try {
                    LaserScan scan = reader.read();
                    if (scan != null && scan.getRanges() != null && listener != null) {
                        float[] ranges = scan.getRanges().getBuffer().array();
                        listener.onLaserScanReceived(ranges, scan.getAngleMin(), scan.getAngleIncrement());
                    }
                } catch (Throwable ignored) {}
            });

            // 5. /map SLAM 점유격자 지도 구독자
            ROS2Topic<OccupancyGrid> mapTopic = new ROS2Topic<>("/map", OccupancyGrid.class);
            mapSubscription = ros2Node.createSubscription(mapTopic, reader -> {
                try {
                    OccupancyGrid map = reader.read();
                    if (map != null && map.getData() != null && listener != null) {
                        byte[] data = map.getData().getBuffer().array();
                        int w = map.getInfo().getWidth();
                        int h = map.getInfo().getHeight();
                        float res = map.getInfo().getResolution();
                        float ox = (float) map.getInfo().getOrigin().getPosition().getX();
                        float oy = (float) map.getInfo().getOrigin().getPosition().getY();
                        listener.onMapReceived(data, w, h, res, ox, oy);
                    }
                } catch (Throwable ignored) {}
            });

            // 6. /amcl_pose Nav2 위치 추정 구독자 (맵 기준 좌표 및 각도)
            ROS2Topic<PoseWithCovarianceStamped> amclTopic = new ROS2Topic<>("/amcl_pose", PoseWithCovarianceStamped.class);
            amclPoseSubscription = ros2Node.createSubscription(amclTopic, reader -> {
                try {
                    PoseWithCovarianceStamped poseMsg = reader.read();
                    if (poseMsg != null && listener != null) {
                        float x = (float) poseMsg.getPose().getPose().getPosition().getX();
                        float y = (float) poseMsg.getPose().getPose().getPosition().getY();
                        double qz = poseMsg.getPose().getPose().getOrientation().getZ();
                        double qw = poseMsg.getPose().getPose().getOrientation().getW();
                        float yaw = (float) (2.0 * Math.atan2(qz, qw));
                        listener.onRobotPoseReceived(x, y, yaw);
                    }
                } catch (Throwable ignored) {}
            });

            // 7. /odom 오도메트리 보조 구독자 (amcl_pose가 없을 때 대비)
            ROS2Topic<Odometry> odomTopic = new ROS2Topic<>("/odom", Odometry.class);
            odomSubscription = ros2Node.createSubscription(odomTopic, reader -> {
                try {
                    Odometry odom = reader.read();
                    if (odom != null && listener != null && amclPoseSubscription == null) {
                        float x = (float) odom.getPose().getPose().getPosition().getX();
                        float y = (float) odom.getPose().getPose().getPosition().getY();
                        double qz = odom.getPose().getPose().getOrientation().getZ();
                        double qw = odom.getPose().getPose().getOrientation().getW();
                        float yaw = (float) (2.0 * Math.atan2(qz, qw));
                        listener.onRobotPoseReceived(x, y, yaw);
                    }
                } catch (Throwable ignored) {}
            });

            // 8. /battery_state 배터리 잔량 구독자
            ROS2Topic<BatteryState> batTopic = new ROS2Topic<>("/battery_state", BatteryState.class);
            batterySubscription = ros2Node.createSubscription(batTopic, reader -> {
                try {
                    BatteryState bat = reader.read();
                    if (bat != null && listener != null) {
                        float pct = bat.getPercentage();
                        if (pct <= 1.0f && pct > 0.0f) pct *= 100f;
                        float volt = bat.getVoltage();
                        boolean isCharging = (bat.getPowerSupplyStatus() == 1);
                        listener.onBatteryStateReceived(pct, volt, isCharging);
                    }
                } catch (Throwable ignored) {}
            });

            // 9. 동적 모니터링 토픽 구독 등록
            switchSubscribedTopic(currentSubTopic);

            isRunning = true;
            if (listener != null) {
                listener.onStatusChanged("DDS 노드 동작 중 (Domain: " + domainId + ")", true);
            }

        } catch (Exception e) {
            Log.e(TAG, "노드 시작 중 예외", e);
            if (listener != null) {
                listener.onStatusChanged("노드 생성 오류: " + e.getMessage(), false);
            }
        }
    }

    public synchronized void publishGoalPose(float worldX, float worldY) {
        if (!isRunning || goalPublisher == null) {
            log("[WARN] 노드가 실행 중이 아닙니다.");
            return;
        }
        try {
            PoseStamped pose = new PoseStamped();
            pose.getHeader().setFrameId("map");
            pose.getPose().getPosition().setX(worldX);
            pose.getPose().getPosition().setY(worldY);
            pose.getPose().getPosition().setZ(0.0);
            pose.getPose().getOrientation().setW(1.0);
            goalPublisher.publish(pose);
            log(String.format(TAG, "[Goal 전송] /goal_pose -> X: %.2f, Y: %.2f", worldX, worldY));
        } catch (Throwable t) {
            log("[ERROR] Goal 전송 오류: " + t.getMessage());
        }
    }

    public synchronized void switchSubscribedTopic(String topicName) {
        this.currentSubTopic = topicName;
        if (!isRunning || ros2Node == null) return;

        try {
            if (topicName.contains("cmd_vel")) {
                ROS2Topic<Twist> topic = new ROS2Topic<>(topicName, Twist.class);
                dynamicSubscription = ros2Node.createSubscription(topic, reader -> {
                    try {
                        Twist msg = reader.read();
                        if (msg != null && listener != null) {
                            String formatted = String.format(
                                    "Linear [x: %.2f, y: %.2f, z: %.2f], Angular [z: %.2f]",
                                    msg.getLinear().getX(), msg.getLinear().getY(), msg.getLinear().getZ(),
                                    msg.getAngular().getZ()
                            );
                            listener.onCustomTopicReceived(topicName, formatted);
                        }
                    } catch (Throwable ignored) {}
                });
            } else if (topicName.contains("scan")) {
                ROS2Topic<LaserScan> topic = new ROS2Topic<>(topicName, LaserScan.class);
                dynamicSubscription = ros2Node.createSubscription(topic, reader -> {
                    try {
                        LaserScan msg = reader.read();
                        if (msg != null && msg.getRanges() != null && listener != null) {
                            int count = msg.getRanges().getBuffer().array().length;
                            String formatted = String.format("포인트: %d개, 범위: [%.2f ~ %.2f]", count, msg.getAngleMin(), msg.getAngleMax());
                            listener.onCustomTopicReceived(topicName, formatted);
                        }
                    } catch (Throwable ignored) {}
                });
            } else {
                ROS2Topic<String_> topic = new ROS2Topic<>(topicName, String_.class);
                dynamicSubscription = ros2Node.createSubscription(topic, reader -> {
                    try {
                        String_ msg = reader.read();
                        if (msg != null && msg.getData() != null && listener != null) {
                            listener.onCustomTopicReceived(topicName, msg.getData().toString());
                        }
                    } catch (Throwable ignored) {}
                });
            }
            log("[Echo Topic 설정]: " + topicName);
        } catch (Throwable e) {
            log("[ERROR] 토픽 전환 실패: " + e.getMessage());
        }
    }

    public synchronized void publishCmdVel(float linearX, float angularZ) {
        if (!isRunning || cmdVelPublisher == null) return;
        try {
            Twist twist = new Twist();
            twist.getLinear().setX(linearX);
            twist.getAngular().setZ(angularZ);
            cmdVelPublisher.publish(twist);
        } catch (Exception e) {
            Log.e(TAG, "cmd_vel 발행 중 예외", e);
        }
    }

    public synchronized void publishStringTopic(String topicName, String text) {
        if (!isRunning || ros2Node == null) {
            log("[WARN] 노드가 실행 중이 아닙니다.");
            return;
        }
        try {
            ROS2Topic<String_> topic = new ROS2Topic<>(topicName, String_.class);
            if (customStringPublisher == null) {
                customStringPublisher = ros2Node.createPublisher(topic);
            }
            String_ msg = new String_();
            msg.setData(text);
            customStringPublisher.publish(msg);
            log("[Pub " + topicName + "]: " + text);
        } catch (Exception e) {
            log("[ERROR] 토픽 발행 실패: " + e.getMessage());
        }
    }

    public synchronized void stopNode() {
        log("stopNode 호출됨");
        isRunning = false;
        if (ros2Node != null) {
            try {
                ros2Node.close();
                log("ROS2Node 종료 완료");
            } catch (Exception e) {
                Log.e(TAG, "ROS2Node 종료 중 오류", e);
            }
            ros2Node = null;
        }
        if (multicastLock != null && multicastLock.isHeld()) {
            multicastLock.release();
            multicastLock = null;
            log("MulticastLock 해제 완료");
        }
        if (listener != null) {
            listener.onStatusChanged("상태: 노드 중지됨", false);
        }
    }

    private void log(String message) {
        if (listener != null) {
            listener.onLog(message);
        }
    }

    public boolean isRunning() {
        return isRunning;
    }
}