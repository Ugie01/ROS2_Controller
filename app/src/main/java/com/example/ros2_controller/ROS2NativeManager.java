package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.wifi.WifiManager;
import android.util.Log;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

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
        void onRobotPoseReceived(float worldX, float worldY, float yaw, boolean isAmcl);
        void onBatteryStateReceived(float percentage, float voltage, boolean isCharging);
        void onCustomTopicReceived(String topic, String data);
        void onServiceResponseReceived(String serviceName, String result);
        void onStatusChanged(String status, boolean isRunning);
        void onLog(String logMessage);
        void onError(String title, String message);
    }

    private final Context context;
    private final ROS2DataListener listener;
    private WifiManager.MulticastLock multicastLock;
    private final ExecutorService serviceThreadPool = Executors.newSingleThreadExecutor();

    private ROS2Node ros2Node;
    private ROS2Publisher<Twist> cmdVelPublisher;
    private ROS2Publisher<PoseStamped> goalPublisher;
    private ROS2Publisher<PoseWithCovarianceStamped> initialPosePublisher;
    private final ConcurrentHashMap<String, ROS2Publisher<String_>> stringPublisherMap = new ConcurrentHashMap<>();

    private ROS2Subscription<CompressedImage> imageSubscription;
    private ROS2Subscription<LaserScan> scanSubscription;
    private ROS2Subscription<OccupancyGrid> mapSubscription;
    private ROS2Subscription<PoseWithCovarianceStamped> amclPoseSubscription;
    private ROS2Subscription<Odometry> odomSubscription;
    private ROS2Subscription<BatteryState> batterySubscription;
    private ROS2Subscription<String_> rqtGraphSubscription; // [추가] RQT 그래프 전용 구독자
    private ROS2Subscription<?> dynamicSubscription;

    private String currentSubTopic = "/chatter";
    private boolean isRunning = false;
    private volatile long lastHeartbeatTime = 0;

    private static final Map<String, String> RESERVED_TOPICS = new HashMap<>();
    static {
        RESERVED_TOPICS.put("/scan", "sensor_msgs/msg/LaserScan");
        RESERVED_TOPICS.put("/cmd_vel", "geometry_msgs/msg/Twist");
        RESERVED_TOPICS.put("/map", "nav_msgs/msg/OccupancyGrid");
        RESERVED_TOPICS.put("/odom", "nav_msgs/msg/Odometry");
        RESERVED_TOPICS.put("/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped");
        RESERVED_TOPICS.put("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped");
        RESERVED_TOPICS.put("/goal_pose", "geometry_msgs/msg/PoseStamped");
        RESERVED_TOPICS.put("/battery_state", "sensor_msgs/msg/BatteryState");
        RESERVED_TOPICS.put("/image_raw/compressed", "sensor_msgs/msg/CompressedImage");
    }

    public ROS2NativeManager(Context context, ROS2DataListener listener) {
        this.context = context;
        this.listener = listener;
    }

    public static boolean isValidRosName(String name) {
        if (name == null || name.trim().isEmpty()) return false;
        String trimmed = name.trim();
        return trimmed.matches("^/[a-zA-Z0-9_/]+$");
    }

    public synchronized void startNode(int domainId, boolean useBestEffortQos) {
        stopNode();

        try {
            WifiManager wifi = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            if (wifi != null) {
                multicastLock = wifi.createMulticastLock("ros2_dds_multicast_lock");
                multicastLock.setReferenceCounted(true);
                multicastLock.acquire();
                log("MulticastLock 획득 완료");
            }

            ros2Node = new ROS2Node("android_controller_node", domainId);
            log("ROS2Node 생성 완료 (Domain: " + domainId + ")");

            cmdVelPublisher = ros2Node.createPublisher(new ROS2Topic<>("/cmd_vel", Twist.class));
            goalPublisher = ros2Node.createPublisher(new ROS2Topic<>("/goal_pose", PoseStamped.class));
            initialPosePublisher = ros2Node.createPublisher(new ROS2Topic<>("/initialpose", PoseWithCovarianceStamped.class));

            imageSubscription = ros2Node.createSubscription(new ROS2Topic<>("/image_raw/compressed", CompressedImage.class), reader -> {
                try {
                    CompressedImage msg = reader.read();
                    if (msg != null && msg.getData() != null) {
                        updateHeartbeat();
                        byte[] data = msg.getData().getBuffer().array();
                        Bitmap bitmap = BitmapFactory.decodeByteArray(data, 0, data.length);
                        if (bitmap != null && listener != null) {
                            listener.onCompressedImageReceived(bitmap);
                        }
                    }
                } catch (Throwable ignored) {}
            });

            scanSubscription = ros2Node.createSubscription(new ROS2Topic<>("/scan", LaserScan.class), reader -> {
                try {
                    LaserScan scan = reader.read();
                    if (scan != null && scan.getRanges() != null && listener != null) {
                        updateHeartbeat();
                        float[] ranges = scan.getRanges().getBuffer().array();
                        listener.onLaserScanReceived(ranges, scan.getAngleMin(), scan.getAngleIncrement());
                    }
                } catch (Throwable ignored) {}
            });

            mapSubscription = ros2Node.createSubscription(new ROS2Topic<>("/map", OccupancyGrid.class), reader -> {
                try {
                    OccupancyGrid map = reader.read();
                    if (map != null && map.getData() != null && listener != null) {
                        updateHeartbeat();
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

            amclPoseSubscription = ros2Node.createSubscription(new ROS2Topic<>("/amcl_pose", PoseWithCovarianceStamped.class), reader -> {
                try {
                    PoseWithCovarianceStamped poseMsg = reader.read();
                    if (poseMsg != null && listener != null) {
                        updateHeartbeat();
                        float x = (float) poseMsg.getPose().getPose().getPosition().getX();
                        float y = (float) poseMsg.getPose().getPose().getPosition().getY();
                        double qz = poseMsg.getPose().getPose().getOrientation().getZ();
                        double qw = poseMsg.getPose().getPose().getOrientation().getW();
                        float yaw = (float) (2.0 * Math.atan2(qz, qw));
                        listener.onRobotPoseReceived(x, y, yaw, true);
                    }
                } catch (Throwable ignored) {}
            });

            odomSubscription = ros2Node.createSubscription(new ROS2Topic<>("/odom", Odometry.class), reader -> {
                try {
                    Odometry odom = reader.read();
                    if (odom != null && listener != null) {
                        updateHeartbeat();
                        float x = (float) odom.getPose().getPose().getPosition().getX();
                        float y = (float) odom.getPose().getPose().getPosition().getY();
                        double qz = odom.getPose().getPose().getOrientation().getZ();
                        double qw = odom.getPose().getPose().getOrientation().getW();
                        float yaw = (float) (2.0 * Math.atan2(qz, qw));
                        listener.onRobotPoseReceived(x, y, yaw, false);
                    }
                } catch (Throwable ignored) {}
            });

            batterySubscription = ros2Node.createSubscription(new ROS2Topic<>("/battery_state", BatteryState.class), reader -> {
                try {
                    BatteryState bat = reader.read();
                    if (bat != null && listener != null) {
                        updateHeartbeat();
                        float pct = bat.getPercentage();
                        if (pct <= 1.0f && pct > 0.0f) pct *= 100f;
                        listener.onBatteryStateReceived(pct, bat.getVoltage(), bat.getPowerSupplyStatus() == 1);
                    }
                } catch (Throwable ignored) {}
            });

            // [핵심 추가] /rqt_graph_data 수신 구독자 등록
            ROS2Topic<String_> rqtTopic = new ROS2Topic<>("/rqt_graph_data", String_.class);
            rqtGraphSubscription = ros2Node.createSubscription(rqtTopic, reader -> {
                try {
                    String_ msg = reader.read();
                    if (msg != null && msg.getData() != null && listener != null) {
                        updateHeartbeat();
                        listener.onCustomTopicReceived("/rqt_graph_data", msg.getData().toString());
                    }
                } catch (Throwable ignored) {}
            });

            switchSubscribedTopic(currentSubTopic);

            isRunning = true;
            updateHeartbeat();
            if (listener != null) {
                listener.onStatusChanged("DDS 노드 동작 중 (Domain: " + domainId + ")", true);
            }

        } catch (Throwable t) {
            Log.e(TAG, "노드 시작 예외", t);
            if (listener != null) {
                listener.onStatusChanged("노드 생성 오류", false);
                listener.onError("노드 생성 실패", "도메인 ID(" + domainId + ")로 ROS 2 노드를 생성하지 못했습니다.\n\n원인: " + t.getMessage());
            }
        }
    }

    private void updateHeartbeat() {
        lastHeartbeatTime = System.currentTimeMillis();
    }

    public long getLastHeartbeatTime() {
        return lastHeartbeatTime;
    }

    public synchronized void publishGoalPose(float worldX, float worldY, float yaw) {
        if (!isRunning || goalPublisher == null) {
            notifyError("Goal 전송 불가", "ROS 2 노드가 실행 중이 아닙니다. 상단에서 [노드 시작]을 먼저 누르세요.");
            return;
        }
        try {
            PoseStamped pose = new PoseStamped();
            pose.getHeader().setFrameId("map");
            pose.getPose().getPosition().setX(worldX);
            pose.getPose().getPosition().setY(worldY);
            pose.getPose().getPosition().setZ(0.0);
            pose.getPose().getOrientation().setZ((float) Math.sin(yaw / 2.0));
            pose.getPose().getOrientation().setW((float) Math.cos(yaw / 2.0));

            goalPublisher.publish(pose);
            log(String.format(Locale.getDefault(), "[Goal 전송] X: %.2f, Y: %.2f, Yaw: %.1f°", worldX, worldY, Math.toDegrees(yaw)));
        } catch (Throwable t) {
            notifyError("Goal 전송 오류", t.getMessage());
        }
    }

    public synchronized void publishInitialPose(float worldX, float worldY, float yaw) {
        if (!isRunning || initialPosePublisher == null) {
            notifyError("위치 추정 전송 불가", "ROS 2 노드가 실행 중이 아닙니다.");
            return;
        }
        try {
            PoseWithCovarianceStamped poseMsg = new PoseWithCovarianceStamped();
            poseMsg.getHeader().setFrameId("map");
            poseMsg.getPose().getPose().getPosition().setX(worldX);
            poseMsg.getPose().getPose().getPosition().setY(worldY);
            poseMsg.getPose().getPose().getPosition().setZ(0.0);
            poseMsg.getPose().getPose().getOrientation().setZ((float) Math.sin(yaw / 2.0));
            poseMsg.getPose().getPose().getOrientation().setW((float) Math.cos(yaw / 2.0));

            initialPosePublisher.publish(poseMsg);

            if (listener != null) {
                listener.onRobotPoseReceived(worldX, worldY, yaw, true);
            }

            log(String.format(Locale.getDefault(), "[2D Pose Estimate] X: %.2f, Y: %.2f, Yaw: %.1f°", worldX, worldY, Math.toDegrees(yaw)));
        } catch (Throwable t) {
            notifyError("2D Pose Estimate 전송 오류", t.getMessage());
        }
    }

    public synchronized void cancelNavigation(float currentRobotX, float currentRobotY, float currentRobotYaw) {
        try {
            publishCmdVel(0f, 0f);
            if (isRunning && goalPublisher != null) {
                publishGoalPose(currentRobotX, currentRobotY, currentRobotYaw);
            }
            log("[Nav2] 자율주행 취소 명령 전송 완료");
        } catch (Throwable t) {
            Log.e(TAG, "Nav 취소 예외", t);
        }
    }

    public synchronized void switchSubscribedTopic(String topicName) {
        if (topicName == null || topicName.trim().isEmpty()) return;
        String trimmed = topicName.trim();
        if (!trimmed.startsWith("/")) trimmed = "/" + trimmed;

        if (!isValidRosName(trimmed)) {
            notifyError("잘못된 토픽 이름", "토픽 이름 '" + topicName + "'은 올바른 ROS 형식이 아닙니다.");
            return;
        }

        this.currentSubTopic = trimmed;
        if (!isRunning || ros2Node == null) return;

        dynamicSubscription = null;

        try {
            final String finalTopic = trimmed;
            if (trimmed.contains("cmd_vel")) {
                dynamicSubscription = ros2Node.createSubscription(new ROS2Topic<>(trimmed, Twist.class), reader -> {
                    try {
                        if (!finalTopic.equals(currentSubTopic)) return;
                        Twist msg = reader.read();
                        if (msg != null && listener != null) {
                            updateHeartbeat();
                            String formatted = String.format(Locale.getDefault(), "Linear [x: %.2f], Angular [z: %.2f]", msg.getLinear().getX(), msg.getAngular().getZ());
                            listener.onCustomTopicReceived(finalTopic, formatted);
                        }
                    } catch (Throwable ignored) {}
                });
            } else if (trimmed.contains("scan")) {
                dynamicSubscription = ros2Node.createSubscription(new ROS2Topic<>(trimmed, LaserScan.class), reader -> {
                    try {
                        if (!finalTopic.equals(currentSubTopic)) return;
                        LaserScan msg = reader.read();
                        if (msg != null && msg.getRanges() != null && listener != null) {
                            updateHeartbeat();
                            listener.onCustomTopicReceived(finalTopic, "LiDAR 포인트: " + msg.getRanges().getBuffer().array().length + "개");
                        }
                    } catch (Throwable ignored) {}
                });
            } else {
                dynamicSubscription = ros2Node.createSubscription(new ROS2Topic<>(trimmed, String_.class), reader -> {
                    try {
                        if (!finalTopic.equals(currentSubTopic)) return;
                        String_ msg = reader.read();
                        if (msg != null && msg.getData() != null && listener != null) {
                            updateHeartbeat();
                            listener.onCustomTopicReceived(finalTopic, msg.getData().toString());
                        }
                    } catch (Throwable ignored) {}
                });
            }
            log("[Echo Topic 설정]: " + trimmed);
        } catch (Throwable t) {
            notifyError("토픽 구독 실패", "토픽 '" + trimmed + "'을 구독할 수 없습니다.\n\n원인: " + t.getMessage());
        }
    }

    public synchronized void publishCmdVel(float linearX, float angularZ) {
        if (!isRunning || cmdVelPublisher == null) return;
        try {
            Twist twist = new Twist();
            twist.getLinear().setX(linearX);
            twist.getAngular().setZ(angularZ);
            cmdVelPublisher.publish(twist);
        } catch (Throwable t) {
            Log.e(TAG, "cmd_vel 발행 예외", t);
        }
    }

    public synchronized void publishStringTopic(String topicName, String text) {
        if (!isRunning || ros2Node == null) {
            notifyError("토픽 발행 불가", "ROS 2 노드가 중지되어 있습니다. 상단에서 [노드 시작]을 먼저 누르세요.");
            return;
        }

        if (topicName == null || topicName.trim().isEmpty()) {
            notifyError("토픽명 누락", "발행할 토픽 이름을 입력하세요.");
            return;
        }

        String validTopic = topicName.trim();
        if (!validTopic.startsWith("/")) validTopic = "/" + validTopic;

        if (!isValidRosName(validTopic)) {
            notifyError("잘못된 토픽 형식", "토픽명 '" + topicName + "'은 올바른 ROS 형식이 아닙니다.");
            return;
        }

        if (RESERVED_TOPICS.containsKey(validTopic)) {
            String requiredType = RESERVED_TOPICS.get(validTopic);
            notifyError("토픽 타입 불일치", "'" + validTopic + "' 토픽은 [" + requiredType + "] 전용 시스템 토픽입니다.");
            return;
        }

        try {
            ROS2Publisher<String_> publisher = stringPublisherMap.get(validTopic);
            if (publisher == null) {
                ROS2Topic<String_> topic = new ROS2Topic<>(validTopic, String_.class);
                publisher = ros2Node.createPublisher(topic);
                stringPublisherMap.put(validTopic, publisher);
            }

            String_ msg = new String_();
            msg.setData(text == null ? "" : text);
            publisher.publish(msg);
            log("[Pub " + validTopic + "]: " + text);
        } catch (Throwable t) {
            notifyError("토픽 발행 오류", "토픽 [" + validTopic + "] 발행 오류: " + t.getMessage());
        }
    }

    public synchronized void stopNode() {
        log("stopNode 호출됨");
        isRunning = false;
        stringPublisherMap.clear();
        dynamicSubscription = null;
        rqtGraphSubscription = null;

        if (ros2Node != null) {
            try {
                ros2Node.close();
                log("ROS2Node 종료 완료");
            } catch (Throwable t) {
                Log.e(TAG, "ROS2Node 종료 오류", t);
            }
            ros2Node = null;
        }
        if (multicastLock != null && multicastLock.isHeld()) {
            try {
                multicastLock.release();
            } catch (Throwable ignored) {}
            multicastLock = null;
        }
        if (listener != null) {
            listener.onStatusChanged("상태: 노드 중지됨", false);
        }
    }

    private void log(String message) {
        if (listener != null) listener.onLog(message);
    }

    public boolean isRunning() {
        return isRunning;
    }

    private void notifyError(String title, String message) {
        log("[ERROR] " + title + ": " + message);
        if (listener != null) {
            listener.onError(title, message);
        }
    }
}