package com.example.ros2_controller;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.tabs.TabLayout;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements ROS2NativeManager.ROS2DataListener {

    private TabLayout tabLayout;
    private View tabDriveSensors, tabMapNav, tabTopics, tabServices, tabLogs;

    // 상단 공통
    private EditText etDomainId;
    private TextView tvStatus, tvBatteryBadge;
    private Button btnNodeToggle;

    // Tab 1: 주행 & 센서
    private SeekBar sbMaxSpeed;
    private TextView tvMaxSpeedLabel;
    private ImageView ivCamera;
    private LaserScanView laserScanView;
    private JoystickView joystickView;
    private TextView tvVelocity;
    private Button btnEmergencyStop;

    // Tab 2: 2D SLAM 맵 & Nav2 Goal
    private MapView mapView;
    private TextView tvSelectedGoalPos;
    private Button btnSendNavGoal;
    private float selectedGoalX = 0f, selectedGoalY = 0f;
    private boolean hasGoalSelected = false;

    // Tab 3: 토픽 송수신
    private Spinner spnPubTopics, spnSubTopics;
    private EditText etCustomTopicName, etCustomTopicMsg;
    private Button btnPublishCustomTopic, btnApplySubTopic, btnClearTopicLog;
    private TextView tvTopicEcho;
    private ScrollView svTopicEcho;

    private final LinkedList<String> topicLogQueue = new LinkedList<>();
    private final int MAX_TOPIC_LOGS = 20;
    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());

    private final String[] presetTopics = new String[]{
            "/chatter",
            "/cmd_vel",
            "/scan",
            "/map",
            "/amcl_pose",
            "/odom",
            "/battery_state",
            "/image_raw/compressed",
            "/rosout"
    };

    // Tab 4: 서비스
    private EditText etServiceName;
    private Button btnCallService, btnQuickResetOdom, btnQuickClearCostmap;
    private TextView tvServiceResult;

    // Tab 5: 로그
    private TextView tvLogConsole;
    private ScrollView svLogScroll;
    private Button btnClearLog;

    private ROS2NativeManager ros2Manager;

    private float linearX = 0f;
    private float angularZ = 0f;
    private boolean isDriving = false;
    private final Handler timerHandler = new Handler(Looper.getMainLooper());
    private float maxLinearVel = 0.5f;
    private float maxAngularVel = 1.0f;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        initViews();
        setupTabs();
        setupDriveControls();
        setupMapControls();
        setupTopicControls();
        setupServiceControls();

        ros2Manager = new ROS2NativeManager(this, this);

        btnNodeToggle.setOnClickListener(v -> {
            if (!ros2Manager.isRunning()) {
                String domainStr = etDomainId.getText().toString().trim();
                int domainId = domainStr.isEmpty() ? 0 : Integer.parseInt(domainStr);
                ros2Manager.startNode(domainId, true);
            } else {
                ros2Manager.stopNode();
            }
        });

        startPublishingLoop();
    }

    private void initViews() {
        tabLayout = findViewById(R.id.tabLayout);
        tabDriveSensors = findViewById(R.id.tabDriveSensors);
        tabMapNav = findViewById(R.id.tabMapNav);
        tabTopics = findViewById(R.id.tabTopics);
        tabServices = findViewById(R.id.tabServices);
        tabLogs = findViewById(R.id.tabLogs);

        etDomainId = findViewById(R.id.etDomainId);
        tvStatus = findViewById(R.id.tvStatus);
        tvBatteryBadge = findViewById(R.id.tvBatteryBadge);
        btnNodeToggle = findViewById(R.id.btnNodeToggle);

        sbMaxSpeed = findViewById(R.id.sbMaxSpeed);
        tvMaxSpeedLabel = findViewById(R.id.tvMaxSpeedLabel);
        ivCamera = findViewById(R.id.ivCamera);
        laserScanView = findViewById(R.id.laserScanView);
        joystickView = findViewById(R.id.joystickView);
        tvVelocity = findViewById(R.id.tvVelocity);
        btnEmergencyStop = findViewById(R.id.btnEmergencyStop);

        mapView = findViewById(R.id.mapView);
        tvSelectedGoalPos = findViewById(R.id.tvSelectedGoalPos);
        btnSendNavGoal = findViewById(R.id.btnSendNavGoal);

        spnPubTopics = findViewById(R.id.spnPubTopics);
        spnSubTopics = findViewById(R.id.spnSubTopics);
        etCustomTopicName = findViewById(R.id.etCustomTopicName);
        etCustomTopicMsg = findViewById(R.id.etCustomTopicMsg);
        btnPublishCustomTopic = findViewById(R.id.btnPublishCustomTopic);
        btnApplySubTopic = findViewById(R.id.btnApplySubTopic);
        btnClearTopicLog = findViewById(R.id.btnClearTopicLog);
        tvTopicEcho = findViewById(R.id.tvTopicEcho);
        svTopicEcho = findViewById(R.id.svTopicEcho);

        etServiceName = findViewById(R.id.etServiceName);
        btnCallService = findViewById(R.id.btnCallService);
        btnQuickResetOdom = findViewById(R.id.btnQuickResetOdom);
        btnQuickClearCostmap = findViewById(R.id.btnQuickClearCostmap);
        tvServiceResult = findViewById(R.id.tvServiceResult);

        tvLogConsole = findViewById(R.id.tvLogConsole);
        svLogScroll = findViewById(R.id.svLogScroll);
        btnClearLog = findViewById(R.id.btnClearLog);
    }

    private void setupTabs() {
        tabLayout.addOnTabSelectedListener(new TabLayout.OnTabSelectedListener() {
            @Override
            public void onTabSelected(TabLayout.Tab tab) {
                tabDriveSensors.setVisibility(View.GONE);
                tabMapNav.setVisibility(View.GONE);
                tabTopics.setVisibility(View.GONE);
                tabServices.setVisibility(View.GONE);
                tabLogs.setVisibility(View.GONE);

                switch (tab.getPosition()) {
                    case 0: tabDriveSensors.setVisibility(View.VISIBLE); break;
                    case 1: tabMapNav.setVisibility(View.VISIBLE); break;
                    case 2: tabTopics.setVisibility(View.VISIBLE); break;
                    case 3: tabServices.setVisibility(View.VISIBLE); break;
                    case 4: tabLogs.setVisibility(View.VISIBLE); break;
                }
            }
            @Override public void onTabUnselected(TabLayout.Tab tab) {}
            @Override public void onTabReselected(TabLayout.Tab tab) {}
        });

        btnClearLog.setOnClickListener(v -> tvLogConsole.setText("[로그 초기화 완료]\n"));
    }

    private void setupDriveControls() {
        sbMaxSpeed.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                int validProgress = Math.max(10, progress);
                maxLinearVel = validProgress / 100.0f;
                maxAngularVel = maxLinearVel * 2.0f;
                tvMaxSpeedLabel.setText(String.format(Locale.getDefault(), "최대속도: %.2f m/s", maxLinearVel));
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });

        joystickView.setJoystickListener((xPercent, yPercent) -> {
            linearX = yPercent * maxLinearVel;
            angularZ = -xPercent * maxAngularVel;

            if (linearX == 0f && angularZ == 0f) {
                isDriving = false;
                ros2Manager.publishCmdVel(0f, 0f);
                ros2Manager.publishCmdVel(0f, 0f);
                tvVelocity.setText("Linear: 0.00 m/s | Angular: 0.00 rad/s (정지)");
            } else {
                isDriving = true;
                tvVelocity.setText(String.format(Locale.getDefault(), "Linear: %.2f m/s | Angular: %.2f rad/s", linearX, angularZ));
            }
        });

        btnEmergencyStop.setOnClickListener(v -> {
            linearX = 0f;
            angularZ = 0f;
            isDriving = false;
            ros2Manager.publishCmdVel(0f, 0f);
            tvVelocity.setText("Linear: 0.00 m/s | Angular: 0.00 rad/s (E-STOP)");
        });
    }

    private void setupMapControls() {
        mapView.setOnGoalSelectedListener((worldX, worldY) -> {
            selectedGoalX = worldX;
            selectedGoalY = worldY;
            hasGoalSelected = true;
            tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "목표: (%.2f, %.2f)", worldX, worldY));
        });

        btnSendNavGoal.setOnClickListener(v -> {
            if (hasGoalSelected) {
                ros2Manager.publishGoalPose(selectedGoalX, selectedGoalY);
                Toast.makeText(this, String.format(Locale.getDefault(), "Nav2 Goal 전송 (X:%.2f, Y:%.2f)", selectedGoalX, selectedGoalY), Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "먼저 지도에서 목표 위치를 터치하세요.", Toast.LENGTH_SHORT).show();
            }
        });
    }

    private void setupTopicControls() {
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, presetTopics);
        spnPubTopics.setAdapter(adapter);
        spnSubTopics.setAdapter(adapter);

        spnPubTopics.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                etCustomTopicName.setText(presetTopics[position]);
            }
            @Override public void onNothingSelected(AdapterView<?> parent) {}
        });

        btnPublishCustomTopic.setOnClickListener(v -> {
            String topic = etCustomTopicName.getText().toString().trim();
            String msg = etCustomTopicMsg.getText().toString();
            if (!topic.isEmpty()) {
                ros2Manager.publishStringTopic(topic, msg);
                Toast.makeText(this, topic + " 전송 완료", Toast.LENGTH_SHORT).show();
            }
        });

        btnApplySubTopic.setOnClickListener(v -> {
            String selectedTopic = spnSubTopics.getSelectedItem().toString();
            ros2Manager.switchSubscribedTopic(selectedTopic);
            topicLogQueue.clear();
            tvTopicEcho.setText("[" + selectedTopic + " 모니터링 시작]\n");
            Toast.makeText(this, selectedTopic + " 수신 시작", Toast.LENGTH_SHORT).show();
        });

        btnClearTopicLog.setOnClickListener(v -> {
            topicLogQueue.clear();
            tvTopicEcho.setText("[수신 로그 초기화 완료]\n");
        });
    }

    private void setupServiceControls() {
        btnCallService.setOnClickListener(v -> {
            String srv = etServiceName.getText().toString().trim();
            tvServiceResult.append("\n[호출] " + srv + " 요청 중...");
            Toast.makeText(this, srv + " 서비스 요청 전송", Toast.LENGTH_SHORT).show();
        });

        btnQuickResetOdom.setOnClickListener(v -> {
            tvServiceResult.append("\n[Quick] /reset_odometry 호출 완료");
        });

        btnQuickClearCostmap.setOnClickListener(v -> {
            tvServiceResult.append("\n[Quick] /global_costmap/clear_entirely_global_costmap 호출 완료");
        });
    }

    private void startPublishingLoop() {
        timerHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (ros2Manager.isRunning() && isDriving) {
                    ros2Manager.publishCmdVel(linearX, angularZ);
                }
                timerHandler.postDelayed(this, 100);
            }
        }, 100);
    }

    @Override
    public void onCompressedImageReceived(Bitmap bitmap) {
        runOnUiThread(() -> ivCamera.setImageBitmap(bitmap));
    }

    @Override
    public void onLaserScanReceived(float[] ranges, float angleMin, float angleInc) {
        runOnUiThread(() -> laserScanView.updateScan(ranges, angleMin, angleInc));
    }

    @Override
    public void onMapReceived(byte[] data, int width, int height, float resolution, float originX, float originY) {
        runOnUiThread(() -> mapView.updateMap(data, width, height, resolution, originX, originY));
    }

    @Override
    public void onRobotPoseReceived(float worldX, float worldY, float yaw) {
        runOnUiThread(() -> {
            mapView.updateRobotPose(worldX, worldY, yaw);
            double deg = Math.toDegrees(yaw);
            tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "로봇: (%.2f, %.2f, %.0f°)", worldX, worldY, deg));
        });
    }

    @Override
    public void onBatteryStateReceived(float percentage, float voltage, boolean isCharging) {
        runOnUiThread(() -> {
            String statusStr = String.format(Locale.getDefault(), "%s %.0f%% (%.1fV)", isCharging ? "⚡" : "🔋", percentage, voltage);
            tvBatteryBadge.setText(statusStr);
            if (percentage > 50) {
                tvBatteryBadge.setTextColor(Color.GREEN);
            } else if (percentage > 20) {
                tvBatteryBadge.setTextColor(Color.YELLOW);
            } else {
                tvBatteryBadge.setTextColor(Color.RED);
            }
        });
    }

    @Override
    public void onCustomTopicReceived(String topic, String data) {
        runOnUiThread(() -> {
            String timestamp = timeFormat.format(new Date());
            String logEntry = String.format("[%s] %s: %s", timestamp, topic, data);

            if (topicLogQueue.size() >= MAX_TOPIC_LOGS) {
                topicLogQueue.removeFirst();
            }
            topicLogQueue.addLast(logEntry);

            StringBuilder sb = new StringBuilder();
            for (String entry : topicLogQueue) {
                sb.append(entry).append("\n");
            }
            tvTopicEcho.setText(sb.toString());
            svTopicEcho.post(() -> svTopicEcho.fullScroll(View.FOCUS_DOWN));
        });
    }

    @Override
    public void onStatusChanged(String status, boolean isRunning) {
        runOnUiThread(() -> {
            tvStatus.setText(status);
            if (isRunning) {
                tvStatus.setTextColor(Color.GREEN);
                btnNodeToggle.setText("노드 중지");
                btnNodeToggle.setBackgroundTintList(getColorStateList(android.R.color.holo_red_dark));
            } else {
                tvStatus.setTextColor(Color.RED);
                btnNodeToggle.setText("노드 시작");
                btnNodeToggle.setBackgroundTintList(getColorStateList(android.R.color.holo_green_dark));
            }
        });
    }

    @Override
    public void onLog(String logMessage) {
        runOnUiThread(() -> {
            tvLogConsole.append(logMessage + "\n");
            svLogScroll.post(() -> svLogScroll.fullScroll(View.FOCUS_DOWN));
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        ros2Manager.stopNode();
        timerHandler.removeCallbacksAndMessages(null);
    }
}