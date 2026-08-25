package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.wifi.WifiManager;
import android.util.Log;

import geometry_msgs.Twist;
import sensor_msgs.CompressedImage;
import sensor_msgs.LaserScan;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

public class ROS2NativeManager {
    private static final String TAG = "ROS2_DEBUG_NativeMgr";

    public interface ROS2DataListener {
        void onCompressedImageReceived(Bitmap bitmap);
        void onLaserScanReceived(float[] ranges, float angleMin, float angleInc);
        void onStatusChanged(String status, boolean isRunning);
    }

    private final Context context;
    private final ROS2DataListener listener;
    private WifiManager.MulticastLock multicastLock;

    private ROS2Node ros2Node;
    private ROS2Publisher<Twist> cmdVelPublisher;
    private ROS2Subscription<CompressedImage> imageSubscription;
    private ROS2Subscription<LaserScan> scanSubscription;

    private boolean isRunning = false;

    public ROS2NativeManager(Context context, ROS2DataListener listener) {
        this.context = context;
        this.listener = listener;
    }

    public synchronized void startNode(int domainId) {
        Log.i(TAG, "startNode 호출됨 - 도메인 ID: " + domainId);
        stopNode();

        try {
            // 1. Wi-Fi Multicast Lock
            WifiManager wifi = (WifiManager) context.getApplicationContext().getSystemService(Context.WIFI_SERVICE);
            if (wifi != null) {
                multicastLock = wifi.createMulticastLock("ros2_dds_multicast_lock");
                multicastLock.setReferenceCounted(true);
                multicastLock.acquire();
                Log.d(TAG, "MulticastLock 획득 완료 (isHeld: " + multicastLock.isHeld() + ")");
            } else {
                Log.w(TAG, "WifiManager를 가져오지 못했습니다.");
            }

            // 2. 노드 생성
            Log.d(TAG, "ROS2Node 생성 중...");
            ros2Node = new ROS2Node("android_controller_node", domainId);
            Log.i(TAG, "ROS2Node 생성 완료: android_controller_node (Domain: " + domainId + ")");

            // 3. /cmd_vel 퍼블리셔
            ROS2Topic<Twist> cmdVelTopic = new ROS2Topic<>("/cmd_vel", Twist.class);
            cmdVelPublisher = ros2Node.createPublisher(cmdVelTopic);
            Log.d(TAG, "Publisher 생성 완료: /cmd_vel");

            // 4. /image_raw/compressed 구독자
            ROS2Topic<CompressedImage> imageTopic = new ROS2Topic<>("/image_raw/compressed", CompressedImage.class);
            imageSubscription = ros2Node.createSubscription(imageTopic, reader -> {
                CompressedImage msg = reader.read();
                if (msg != null && msg.getData() != null) {
                    byte[] data = msg.getData().getBuffer().array();
                    Log.d(TAG, "[Topic Sub] 이미지 수신 완료 - 데이터 크기: " + data.length + " bytes");
                    Bitmap bitmap = BitmapFactory.decodeByteArray(data, 0, data.length);
                    if (bitmap != null && listener != null) {
                        listener.onCompressedImageReceived(bitmap);
                    } else if (bitmap == null) {
                        Log.w(TAG, "이미지 디코딩 실패 (Bitmap == null)");
                    }
                }
            });
            Log.d(TAG, "Subscription 생성 완료: /image_raw/compressed");

            // 5. /scan LiDAR 구독자
            ROS2Topic<LaserScan> scanTopic = new ROS2Topic<>("/scan", LaserScan.class);
            scanSubscription = ros2Node.createSubscription(scanTopic, reader -> {
                LaserScan scan = reader.read();
                if (scan != null && scan.getRanges() != null && listener != null) {
                    float[] ranges = scan.getRanges().getBuffer().array();
                    Log.d(TAG, "[Topic Sub] LaserScan 수신 완료 - 포인트 개수: " + ranges.length);
                    listener.onLaserScanReceived(ranges, scan.getAngleMin(), scan.getAngleIncrement());
                }
            });
            Log.d(TAG, "Subscription 생성 완료: /scan");

            isRunning = true;
            if (listener != null) {
                listener.onStatusChanged("DDS 노드 동작 중 (Domain: " + domainId + ")", true);
            }

        } catch (Exception e) {
            Log.e(TAG, "노드 시작 중 예외 발생", e);
            if (listener != null) {
                listener.onStatusChanged("노드 생성 오류: " + e.getMessage(), false);
            }
        }
    }

    public synchronized void publishCmdVel(float linearX, float angularZ) {
        if (!isRunning || cmdVelPublisher == null) {
            Log.w(TAG, "publishCmdVel 실패: 노드가 실행 중이 아니거나 Publisher가 null입니다.");
            return;
        }
        try {
            Twist twist = new Twist();
            twist.getLinear().setX(linearX);
            twist.getAngular().setZ(angularZ);
            cmdVelPublisher.publish(twist);
            Log.d(TAG, String.format("[Topic Pub] /cmd_vel 발행 -> Linear: %.2f, Angular: %.2f", linearX, angularZ));
        } catch (Exception e) {
            Log.e(TAG, "cmd_vel 발행 중 예외 발생", e);
        }
    }

    public synchronized void stopNode() {
        Log.i(TAG, "stopNode 호출됨");
        isRunning = false;
        if (ros2Node != null) {
            try {
                ros2Node.close();
                Log.d(TAG, "ROS2Node 종료 완료");
            } catch (Exception e) {
                Log.e(TAG, "ROS2Node 종료 중 오류", e);
            }
            ros2Node = null;
        }
        if (multicastLock != null && multicastLock.isHeld()) {
            multicastLock.release();
            multicastLock = null;
            Log.d(TAG, "MulticastLock 해제 완료");
        }
        if (listener != null) {
            listener.onStatusChanged("상태: 노드 중지됨", false);
        }
    }

    public boolean isRunning() {
        return isRunning;
    }
}