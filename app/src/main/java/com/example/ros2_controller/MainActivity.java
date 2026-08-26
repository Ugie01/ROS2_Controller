package com.example.ros2_controller;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
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
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import com.google.android.material.tabs.TabLayout;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

public class MainActivity extends AppCompatActivity implements ROS2NativeManager.ROS2DataListener {

    private TabLayout tabLayout;
    private View tabDriveSensors, tabSlamMapping, tabNav2Drive, tabTopics, tabServices, tabRqtGraph, tabLogs;

    // 상단 공통
    private EditText etDomainId;
    private Spinner spnGlobalQos;
    private TextView tvStatus, tvBatteryBadge;
    private Button btnNodeToggle;

    private final String[] qosPresets = new String[]{
            "TransientLocal (맵 최적)",
            "Reliable (신뢰성)",
            "BestEffort (센서/속도)"
    };

    // Tab 1: 주행 & 센서
    private SeekBar sbMaxSpeed;
    private TextView tvMaxSpeedLabel;
    private ImageView ivCamera;
    private LaserScanView laserScanView;
    private JoystickView joystickView;
    private TextView tvVelocity;
    private Button btnEmergencyStop;

    // Tab 2: SLAM 맵핑 전용
    private MapView mapViewSlam;
    private JoystickView mapJoystickView;
    private Button btnSaveMapFromSlam;

    // Tab 3: Nav2 자율주행 전용
    private MapView mapViewNav2;
    private TextView tvSelectedGoalPos;
    private Button btnModeSingleGoal, btnModeWaypoints, btnModeInitialPose, btnCancelNav, btnSendNavGoal, btnClearMapMarkers;

    private int currentMapMode = MapView.MODE_SINGLE_GOAL;
    private float selectedGoalX = 0f, selectedGoalY = 0f, selectedGoalYaw = 0f;
    private float selectedInitX = 0f, selectedInitY = 0f, selectedInitYaw = 0f;
    private boolean hasGoalSelected = false;
    private boolean hasInitPoseSelected = false;
    private boolean isNavigating = false;
    private boolean isPatrolling = false;
    private float curRobotX = 0f, curRobotY = 0f, curRobotYaw = 0f;

    private byte[] currentMapData;
    private int currentMapW = 0, currentMapH = 0;
    private float currentMapRes = 0.05f, currentMapOx = 0f, currentMapOy = 0f;

    // Tab 4: 토픽 송수신 (/chatter)
    private EditText etChatterMsg;
    private Button btnPublishChatter, btnApplySubTopic, btnClearTopicLog;
    private Spinner spnSubTopics;
    private TextView tvTopicEcho;
    private ScrollView svTopicEcho;

    private final LinkedList<String> topicLogQueue = new LinkedList<>();
    private final int MAX_TOPIC_LOGS = 20;
    private final SimpleDateFormat timeFormat = new SimpleDateFormat("HH:mm:ss.SSS", Locale.getDefault());

    private final String[] presetSubTopics = new String[]{
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

    // Tab 5: JSON 파라미터 튜닝 (/param_set_cmd)
    private Spinner spnParamNodes, spnParams;
    private EditText etParamValue, etPresetFileName;
    private Button btnSetParam, btnSaveParamsPreset, btnLoadParamsPreset;
    private TextView tvParamLog;
    private ScrollView svParamLog;

    private final Map<String, String[]> nodeParamsMap = new HashMap<>();
    private final Map<String, String> currentParamValues = new HashMap<>();

    // Tab 6: RQT 그래프
    private RqtGraphView rqtGraphView;
    private Button btnRefreshRqtGraph;

    // Tab 7: 로그
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
        setupGlobalCrashGuard();

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
        setupSlamControls();
        setupNav2Controls();
        setupTopicControls();
        setupParamControls();
        setupRqtGraphControls();

        ros2Manager = new ROS2NativeManager(this, this);

        btnNodeToggle.setOnClickListener(v -> {
            try {
                if (!ros2Manager.isRunning()) {
                    String domainStr = etDomainId.getText().toString().trim();
                    int domainId = 0;
                    if (!domainStr.isEmpty()) {
                        try {
                            domainId = Integer.parseInt(domainStr);
                        } catch (NumberFormatException nfe) {
                            showErrorDialog("도메인 ID 오류", "도메인 ID는 0 이상의 정수만 가능합니다.");
                            return;
                        }
                    }
                    boolean useBestEffort = (spnGlobalQos.getSelectedItemPosition() == 2);
                    ros2Manager.startNode(domainId, useBestEffort);
                } else {
                    ros2Manager.stopNode();
                }
            } catch (Throwable t) {
                showErrorDialog("노드 제어 오류", t.getMessage());
            }
        });

        startPublishingLoop();
        startHeartbeatWatchdog();
    }

    private void setupGlobalCrashGuard() {
        Thread.setDefaultUncaughtExceptionHandler((thread, throwable) -> {
            Log.e("ROS2_GUARD", "백그라운드 스레드 예외: " + throwable.getMessage(), throwable);
            runOnUiThread(() -> showErrorDialog("백그라운드 오류", throwable.getMessage()));
        });

        new Handler(Looper.getMainLooper()).post(() -> {
            while (true) {
                try {
                    Looper.loop();
                } catch (Throwable t) {
                    Log.e("ROS2_GUARD", "메인 루퍼 예외: " + t.getMessage(), t);
                    showErrorDialog("시스템 예외 복구됨", t.getMessage());
                }
            }
        });
    }

    public void showErrorDialog(String title, String message) {
        runOnUiThread(() -> {
            try {
                if (isFinishing() || isDestroyed()) return;
                new AlertDialog.Builder(this)
                        .setTitle("⚠️ " + title)
                        .setMessage(message)
                        .setPositiveButton("확인", (dialog, which) -> dialog.dismiss())
                        .setCancelable(true)
                        .show();
            } catch (Throwable ignored) {}
        });
    }

    @Override
    public void onError(String title, String message) {
        showErrorDialog(title, message);
    }

    private void initViews() {
        tabLayout = findViewById(R.id.tabLayout);
        tabDriveSensors = findViewById(R.id.tabDriveSensors);
        tabSlamMapping = findViewById(R.id.tabSlamMapping);
        tabNav2Drive = findViewById(R.id.tabNav2Drive);
        tabTopics = findViewById(R.id.tabTopics);
        tabServices = findViewById(R.id.tabServices);
        tabRqtGraph = findViewById(R.id.tabRqtGraph);
        tabLogs = findViewById(R.id.tabLogs);

        etDomainId = findViewById(R.id.etDomainId);
        spnGlobalQos = findViewById(R.id.spnGlobalQos);
        tvStatus = findViewById(R.id.tvStatus);
        tvBatteryBadge = findViewById(R.id.tvBatteryBadge);
        btnNodeToggle = findViewById(R.id.btnNodeToggle);

        ArrayAdapter<String> qosAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, qosPresets);
        qosAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spnGlobalQos.setAdapter(qosAdapter);

        // Tab 1
        sbMaxSpeed = findViewById(R.id.sbMaxSpeed);
        tvMaxSpeedLabel = findViewById(R.id.tvMaxSpeedLabel);
        ivCamera = findViewById(R.id.ivCamera);
        laserScanView = findViewById(R.id.laserScanView);
        joystickView = findViewById(R.id.joystickView);
        tvVelocity = findViewById(R.id.tvVelocity);
        btnEmergencyStop = findViewById(R.id.btnEmergencyStop);

        // Tab 2 (SLAM)
        mapViewSlam = findViewById(R.id.mapViewSlam);
        mapJoystickView = findViewById(R.id.mapJoystickView);
        btnSaveMapFromSlam = findViewById(R.id.btnSaveMapFromSlam);

        // Tab 3 (Nav2)
        mapViewNav2 = findViewById(R.id.mapViewNav2);
        tvSelectedGoalPos = findViewById(R.id.tvSelectedGoalPos);
        btnModeSingleGoal = findViewById(R.id.btnModeSingleGoal);
        btnModeWaypoints = findViewById(R.id.btnModeWaypoints);
        btnModeInitialPose = findViewById(R.id.btnModeInitialPose);
        btnCancelNav = findViewById(R.id.btnCancelNav);
        btnSendNavGoal = findViewById(R.id.btnSendNavGoal);
        btnClearMapMarkers = findViewById(R.id.btnClearMapMarkers);

        // Tab 4 (Topic)
        etChatterMsg = findViewById(R.id.etChatterMsg);
        btnPublishChatter = findViewById(R.id.btnPublishChatter);
        spnSubTopics = findViewById(R.id.spnSubTopics);
        btnApplySubTopic = findViewById(R.id.btnApplySubTopic);
        btnClearTopicLog = findViewById(R.id.btnClearTopicLog);
        tvTopicEcho = findViewById(R.id.tvTopicEcho);
        svTopicEcho = findViewById(R.id.svTopicEcho);

        // Tab 5 (Params)
        spnParamNodes = findViewById(R.id.spnParamNodes);
        spnParams = findViewById(R.id.spnParams);
        etParamValue = findViewById(R.id.etParamValue);
        btnSetParam = findViewById(R.id.btnSetParam);
        etPresetFileName = findViewById(R.id.etPresetFileName);
        btnSaveParamsPreset = findViewById(R.id.btnSaveParamsPreset);
        btnLoadParamsPreset = findViewById(R.id.btnLoadParamsPreset);
        tvParamLog = findViewById(R.id.tvParamLog);
        svParamLog = findViewById(R.id.svParamLog);

        // Tab 6 (RQT Graph)
        rqtGraphView = findViewById(R.id.rqtGraphView);
        btnRefreshRqtGraph = findViewById(R.id.btnRefreshRqtGraph);

        // Tab 7 (Logs)
        tvLogConsole = findViewById(R.id.tvLogConsole);
        svLogScroll = findViewById(R.id.svLogScroll);
        btnClearLog = findViewById(R.id.btnClearLog);
    }

    /**
     * [핵심] 탭 전환 시 Nav2 맵과 SLAM 맵에 캐시된 맵/위치를 즉시 렌더링
     */
    private void setupTabs() {
        tabLayout.addOnTabSelectedListener(new TabLayout.OnTabSelectedListener() {
            @Override
            public void onTabSelected(TabLayout.Tab tab) {
                tabDriveSensors.setVisibility(View.GONE);
                tabSlamMapping.setVisibility(View.GONE);
                tabNav2Drive.setVisibility(View.GONE);
                tabTopics.setVisibility(View.GONE);
                tabServices.setVisibility(View.GONE);
                tabRqtGraph.setVisibility(View.GONE);
                tabLogs.setVisibility(View.GONE);

                switch (tab.getPosition()) {
                    case 0:
                        tabDriveSensors.setVisibility(View.VISIBLE);
                        break;
                    case 1:
                        tabSlamMapping.setVisibility(View.VISIBLE);
                        if (currentMapData != null && mapViewSlam != null) {
                            mapViewSlam.updateMap(currentMapData, currentMapW, currentMapH, currentMapRes, currentMapOx, currentMapOy);
                            mapViewSlam.updateRobotPose(curRobotX, curRobotY, curRobotYaw);
                        }
                        break;
                    case 2:
                        // Nav2 탭 선택 시 즉시 맵과 로봇 위치를 주입하여 검은 화면 방지
                        tabNav2Drive.setVisibility(View.VISIBLE);
                        if (currentMapData != null && mapViewNav2 != null) {
                            mapViewNav2.updateMap(currentMapData, currentMapW, currentMapH, currentMapRes, currentMapOx, currentMapOy);
                            mapViewNav2.updateRobotPose(curRobotX, curRobotY, curRobotYaw);
                        }
                        break;
                    case 3:
                        tabTopics.setVisibility(View.VISIBLE);
                        break;
                    case 4:
                        tabServices.setVisibility(View.VISIBLE);
                        break;
                    case 5:
                        tabRqtGraph.setVisibility(View.VISIBLE);
                        if (rqtGraphView != null) rqtGraphView.resetView();
                        break;
                    case 6:
                        tabLogs.setVisibility(View.VISIBLE);
                        break;
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

        JoystickView.JoystickListener driveListener = (xPercent, yPercent) -> {
            linearX = yPercent * maxLinearVel;
            angularZ = -xPercent * maxAngularVel;

            if (linearX == 0f && angularZ == 0f) {
                isDriving = false;
                ros2Manager.publishCmdVel(0f, 0f);
                tvVelocity.setText("Linear: 0.00 m/s | Angular: 0.00 rad/s (정지)");
            } else {
                isDriving = true;
                tvVelocity.setText(String.format(Locale.getDefault(), "Linear: %.2f m/s | Angular: %.2f rad/s", linearX, angularZ));
            }
        };

        joystickView.setJoystickListener(driveListener);
        mapJoystickView.setJoystickListener(driveListener);

        btnEmergencyStop.setOnClickListener(v -> {
            linearX = 0f;
            angularZ = 0f;
            isDriving = false;
            ros2Manager.publishCmdVel(0f, 0f);
            tvVelocity.setText("Linear: 0.00 m/s | Angular: 0.00 rad/s (E-STOP)");
        });
    }

    private void setupSlamControls() {
        btnSaveMapFromSlam.setOnClickListener(v -> {
            String defaultMapName = "my_map_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault()).format(new Date());
            ros2Manager.publishStringTopic("/save_map_cmd", defaultMapName);
            if (currentMapData != null) {
                MapSaverLocal.saveMapToDevice(this, currentMapData, currentMapW, currentMapH, currentMapRes, currentMapOx, currentMapOy, defaultMapName);
            }
            showErrorDialog("맵 저장 요청 완료", "PC의 ROS 2 실행 폴더에 맵 저장을 요청했습니다!\n\n파일명: " + defaultMapName + ".yaml / .pgm");
        });
    }

    private void setupNav2Controls() {
        mapViewNav2.setOnMapEventListener(new MapView.OnMapEventListener() {
            @Override
            public void onSingleGoalSelected(float worldX, float worldY, float yaw) {
                selectedGoalX = worldX;
                selectedGoalY = worldY;
                selectedGoalYaw = yaw;
                hasGoalSelected = true;
                tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "목표: (X:%.2f, Y:%.2f, %.0f°)", worldX, worldY, Math.toDegrees(yaw)));
            }

            @Override
            public void onInitialPoseSelected(float worldX, float worldY, float yaw) {
                selectedInitX = worldX;
                selectedInitY = worldY;
                selectedInitYaw = yaw;
                hasInitPoseSelected = true;
                tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "추정 위치: (X:%.2f, Y:%.2f, %.0f°)", worldX, worldY, Math.toDegrees(yaw)));
            }

            @Override
            public void onWaypointsUpdated(int count) {
                tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "경유지 %d개 등록됨", count));
            }
        });

        btnModeSingleGoal.setOnClickListener(v -> {
            if (isNavigating || isPatrolling) return;
            currentMapMode = MapView.MODE_SINGLE_GOAL;
            mapViewNav2.setMode(MapView.MODE_SINGLE_GOAL);
            updateModeButtons(btnModeSingleGoal);
            btnSendNavGoal.setText("Goal 전송");
            tvSelectedGoalPos.setText("단일 목표 모드: 지도 터치&드래그로 목표 설정");
        });

        btnModeWaypoints.setOnClickListener(v -> {
            if (isNavigating || isPatrolling) return;
            currentMapMode = MapView.MODE_WAYPOINTS;
            mapViewNav2.setMode(MapView.MODE_WAYPOINTS);
            updateModeButtons(btnModeWaypoints);
            btnSendNavGoal.setText("순찰 시작");
            tvSelectedGoalPos.setText("경유지 순찰 모드: 터치로 지점 추가");
        });

        btnModeInitialPose.setOnClickListener(v -> {
            if (isNavigating || isPatrolling) return;
            currentMapMode = MapView.MODE_INITIAL_POSE;
            mapViewNav2.setMode(MapView.MODE_INITIAL_POSE);
            updateModeButtons(btnModeInitialPose);
            btnSendNavGoal.setText("위치 적용");
            tvSelectedGoalPos.setText("2D 위치추정 모드: 실제 로봇 위치&방향 터치/드래그");
        });

        btnCancelNav.setOnClickListener(v -> {
            isNavigating = false;
            isPatrolling = false;
            hasGoalSelected = false;
            hasInitPoseSelected = false;
            mapViewNav2.clearAllMarkers();
            ros2Manager.cancelNavigation(curRobotX, curRobotY, curRobotYaw);
            Toast.makeText(this, "자율주행 취소됨", Toast.LENGTH_SHORT).show();
            tvSelectedGoalPos.setText("자율주행 취소됨 (정지 상태)");
        });

        btnSendNavGoal.setOnClickListener(v -> {
            if (currentMapMode == MapView.MODE_INITIAL_POSE) {
                if (hasInitPoseSelected) {
                    ros2Manager.publishInitialPose(selectedInitX, selectedInitY, selectedInitYaw);
                    Toast.makeText(this, "2D Pose Estimate 전송 완료 (/initialpose)", Toast.LENGTH_SHORT).show();
                    mapViewNav2.clearAllMarkers();
                    hasInitPoseSelected = false;
                } else {
                    showErrorDialog("위치 미지정", "지도에서 로봇의 실제 위치와 방향을 드래그하세요.");
                }
            } else if (currentMapMode == MapView.MODE_WAYPOINTS) {
                List<MapView.Waypoint> wps = mapViewNav2.getWaypoints();
                if (!wps.isEmpty()) {
                    isPatrolling = true;
                    isNavigating = true;
                    sendCurrentWaypointGoal();
                    Toast.makeText(this, "경유지 순찰 시작", Toast.LENGTH_SHORT).show();
                } else {
                    showErrorDialog("경유지 없음", "경유지를 1개 이상 추가하세요.");
                }
            } else {
                if (hasGoalSelected) {
                    isNavigating = true;
                    ros2Manager.publishGoalPose(selectedGoalX, selectedGoalY, selectedGoalYaw);
                    Toast.makeText(this, "Nav2 Goal 전송 완료", Toast.LENGTH_SHORT).show();
                } else {
                    showErrorDialog("목표 미지정", "지도에서 목표 위치를 터치&드래그하세요.");
                }
            }
        });

        btnClearMapMarkers.setOnClickListener(v -> {
            isNavigating = false;
            isPatrolling = false;
            hasGoalSelected = false;
            hasInitPoseSelected = false;
            mapViewNav2.clearAllMarkers();
            tvSelectedGoalPos.setText("지도 마커 초기화 완료");
        });
    }

    private void updateModeButtons(Button activeBtn) {
        btnModeSingleGoal.setBackgroundTintList(getColorStateList(android.R.color.darker_gray));
        btnModeSingleGoal.setTextColor(Color.WHITE);
        btnModeWaypoints.setBackgroundTintList(getColorStateList(android.R.color.darker_gray));
        btnModeWaypoints.setTextColor(Color.WHITE);
        btnModeInitialPose.setBackgroundTintList(getColorStateList(android.R.color.darker_gray));
        btnModeInitialPose.setTextColor(Color.WHITE);

        activeBtn.setBackgroundTintList(getColorStateList(android.R.color.holo_green_dark));
        activeBtn.setTextColor(Color.BLACK);
    }

    private void sendCurrentWaypointGoal() {
        List<MapView.Waypoint> wps = mapViewNav2.getWaypoints();
        if (isPatrolling && !wps.isEmpty()) {
            MapView.Waypoint targetWp = wps.get(0);
            ros2Manager.publishGoalPose(targetWp.x, targetWp.y, targetWp.yaw);
            tvSelectedGoalPos.setText(String.format(Locale.getDefault(), "순찰 중: 남은 경유지 %d개", wps.size()));
        }
    }

    private void setupTopicControls() {
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, presetSubTopics) {
            @Override
            public View getView(int position, View convertView, android.view.ViewGroup parent) {
                View view = super.getView(position, convertView, parent);
                if (view instanceof TextView) {
                    ((TextView) view).setTextColor(Color.WHITE);
                    ((TextView) view).setTextSize(12);
                }
                return view;
            }
        };
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spnSubTopics.setAdapter(adapter);

        btnPublishChatter.setOnClickListener(v -> {
            String msg = etChatterMsg.getText().toString();
            ros2Manager.publishStringTopic("/chatter", msg);
            Toast.makeText(this, "/chatter 전송 완료", Toast.LENGTH_SHORT).show();
        });

        btnApplySubTopic.setOnClickListener(v -> {
            if (spnSubTopics.getSelectedItem() != null) {
                String selectedTopic = spnSubTopics.getSelectedItem().toString();
                ros2Manager.switchSubscribedTopic(selectedTopic);
                topicLogQueue.clear();
                tvTopicEcho.setText("[" + selectedTopic + " 모니터링 시작]\n");
            }
        });

        btnClearTopicLog.setOnClickListener(v -> {
            topicLogQueue.clear();
            tvTopicEcho.setText("[수신 로그 초기화 완료]\n");
        });
    }

    private void setupParamControls() {
        nodeParamsMap.put("/controller_server", new String[]{
                "FollowPath.desired_linear_vel", "FollowPath.max_angular_accel", "FollowPath.lookahead_dist", "FollowPath.min_approach_linear_velocity", "controller_frequency"
        });
        nodeParamsMap.put("/amcl", new String[]{
                "max_particles", "min_particles", "update_min_d", "update_min_a", "laser_max_range", "transform_tolerance"
        });
        nodeParamsMap.put("/local_costmap/local_costmap", new String[]{
                "inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor", "update_frequency"
        });
        nodeParamsMap.put("/global_costmap/global_costmap", new String[]{
                "inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor", "update_frequency"
        });
        nodeParamsMap.put("/planner_server", new String[]{
                "GridBased.tolerance", "expected_planner_frequency"
        });

        currentParamValues.put("/controller_server.FollowPath.desired_linear_vel", "0.20");
        currentParamValues.put("/controller_server.FollowPath.lookahead_dist", "0.60");
        currentParamValues.put("/amcl.max_particles", "2000");
        currentParamValues.put("/amcl.min_particles", "500");
        currentParamValues.put("/local_costmap/local_costmap.inflation_layer.inflation_radius", "0.12");

        String[] nodes = nodeParamsMap.keySet().toArray(new String[0]);
        ArrayAdapter<String> nodeAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, nodes);
        nodeAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spnParamNodes.setAdapter(nodeAdapter);

        spnParamNodes.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (view instanceof TextView) ((TextView) view).setTextColor(Color.WHITE);
                String selectedNode = nodes[position];
                String[] params = nodeParamsMap.get(selectedNode);
                if (params != null) {
                    ArrayAdapter<String> paramAdapter = new ArrayAdapter<>(MainActivity.this, android.R.layout.simple_spinner_item, params);
                    paramAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
                    spnParams.setAdapter(paramAdapter);
                }
            }
            @Override public void onNothingSelected(AdapterView<?> parent) {}
        });

        spnParams.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (view instanceof TextView) ((TextView) view).setTextColor(Color.WHITE);
                String fullKey = spnParamNodes.getSelectedItem().toString() + "." + spnParams.getSelectedItem().toString();
                String val = currentParamValues.getOrDefault(fullKey, "0.0");
                etParamValue.setText(val);
            }
            @Override public void onNothingSelected(AdapterView<?> parent) {}
        });

        btnSetParam.setOnClickListener(v -> {
            String node = spnParamNodes.getSelectedItem().toString();
            String param = spnParams.getSelectedItem().toString();
            String fullKey = node + "." + param;
            String oldVal = currentParamValues.getOrDefault(fullKey, "(기본값)");
            String newVal = etParamValue.getText().toString().trim();

            if (newVal.isEmpty()) {
                showErrorDialog("입력 오류", "변경할 값을 입력하세요.");
                return;
            }

            currentParamValues.put(fullKey, newVal);

            try {
                JSONObject json = new JSONObject();
                json.put("node", node);
                json.put("param", param);
                json.put("value", newVal);
                String jsonStr = json.toString();

                ros2Manager.publishStringTopic("/param_set_cmd", jsonStr);

                String timestamp = timeFormat.format(new Date());
                String logEntry = String.format(Locale.getDefault(), "[%s] %s\n  ➔ 변경: %s ➔ %s\n  ➔ JSON 전송: %s\n\n", timestamp, fullKey, oldVal, newVal, jsonStr);
                tvParamLog.append(logEntry);
                svParamLog.post(() -> svParamLog.fullScroll(View.FOCUS_DOWN));

                Toast.makeText(this, "파라미터 변경 요청 전송 완료", Toast.LENGTH_SHORT).show();
            } catch (Exception e) {
                showErrorDialog("전송 오류", e.getMessage());
            }
        });

        btnSaveParamsPreset.setOnClickListener(v -> {
            String fileName = etPresetFileName.getText().toString().trim();
            if (fileName.isEmpty()) fileName = "params_preset";
            if (!fileName.endsWith(".json")) fileName += ".json";

            try {
                JSONObject json = new JSONObject(currentParamValues);
                File file = new File(getFilesDir(), fileName);
                FileWriter writer = new FileWriter(file);
                writer.write(json.toString(2));
                writer.flush();
                writer.close();

                showErrorDialog("프리셋 저장 완료", "파라미터 설정이 저장되었습니다.\n파일명: " + fileName);
            } catch (Exception e) {
                showErrorDialog("저장 실패", e.getMessage());
            }
        });

        btnLoadParamsPreset.setOnClickListener(v -> {
            String fileName = etPresetFileName.getText().toString().trim();
            if (fileName.isEmpty()) fileName = "params_preset";
            if (!fileName.endsWith(".json")) fileName += ".json";

            File file = new File(getFilesDir(), fileName);
            if (!file.exists()) {
                showErrorDialog("파일 없음", "저장된 프리셋 파일(" + fileName + ")을 찾을 수 없습니다.");
                return;
            }

            try {
                BufferedReader br = new BufferedReader(new FileReader(file));
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) sb.append(line);
                br.close();

                JSONObject json = new JSONObject(sb.toString());
                tvParamLog.append("\n=== [프리셋 파일 로드: " + fileName + "] ===\n");
                Iterator<String> keys = json.keys();
                while (keys.hasNext()) {
                    String key = keys.next();
                    String val = json.getString(key);
                    String old = currentParamValues.getOrDefault(key, "(없음)");
                    currentParamValues.put(key, val);
                    tvParamLog.append(String.format("  • %s: %s ➔ %s\n", key, old, val));
                }
                svParamLog.post(() -> svParamLog.fullScroll(View.FOCUS_DOWN));
                Toast.makeText(this, "프리셋 로드 완료", Toast.LENGTH_SHORT).show();
            } catch (Exception e) {
                showErrorDialog("불러오기 실패", e.getMessage());
            }
        });
    }

    /**
     * [Tab 6: RQT 새로고침 시 1.0배율 기본 크기/위치로 리셋]
     */
    private void setupRqtGraphControls() {
        btnRefreshRqtGraph.setOnClickListener(v -> {
            if (rqtGraphView != null) {
                rqtGraphView.resetView(); // 기본 크기 및 중앙 정렬 원복
            }
            ros2Manager.publishStringTopic("/request_rqt_graph", "refresh");
            Toast.makeText(this, "RQT 그래프 새로고침 및 뷰 초기화 완료", Toast.LENGTH_SHORT).show();
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

    private void startHeartbeatWatchdog() {
        timerHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (ros2Manager.isRunning()) {
                    long elapsed = System.currentTimeMillis() - ros2Manager.getLastHeartbeatTime();
                    if (elapsed > 2500 && ros2Manager.getLastHeartbeatTime() > 0) {
                        isDriving = false;
                        ros2Manager.publishCmdVel(0f, 0f);
                        tvStatus.setText("⚠️ 통신 두절 (Fail-Safe)");
                        tvStatus.setTextColor(Color.parseColor("#FF9800"));
                    } else if (elapsed <= 2500) {
                        tvStatus.setText("● 정상 연결됨");
                        tvStatus.setTextColor(Color.GREEN);
                    }
                }
                timerHandler.postDelayed(this, 1000);
            }
        }, 1000);
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
        runOnUiThread(() -> {
            this.currentMapData = data;
            this.currentMapW = width;
            this.currentMapH = height;
            this.currentMapRes = resolution;
            this.currentMapOx = originX;
            this.currentMapOy = originY;

            if (mapViewSlam != null) mapViewSlam.updateMap(data, width, height, resolution, originX, originY);
            if (mapViewNav2 != null) mapViewNav2.updateMap(data, width, height, resolution, originX, originY);
        });
    }

    @Override
    public void onRobotPoseReceived(float worldX, float worldY, float yaw, boolean isAmcl) {
        runOnUiThread(() -> {
            curRobotX = worldX;
            curRobotY = worldY;
            curRobotYaw = yaw;

            // 1. SLAM 맵핑 탭: Odom이든 AMCL이든 들어오는 최신 위치를 100% 즉시 반영
            if (mapViewSlam != null) {
                mapViewSlam.updateRobotPose(worldX, worldY, yaw);
            }

            // 2. Nav2 자율주행 탭: AMCL 데이터일 때만 업데이트 (정지 시 (0,0) 덮어쓰기 원천 차단)
            if (isAmcl && mapViewNav2 != null) {
                mapViewNav2.updateRobotPose(worldX, worldY, yaw);
            }

            // 경유지 순찰 거리 계산
            if (isPatrolling && mapViewNav2 != null) {
                List<MapView.Waypoint> wps = mapViewNav2.getWaypoints();
                if (!wps.isEmpty()) {
                    MapView.Waypoint currentTarget = wps.get(0);
                    double dist = Math.hypot(worldX - currentTarget.x, worldY - currentTarget.y);
                    if (dist < 0.35) {
                        mapViewNav2.removeFirstWaypoint();
                        if (!mapViewNav2.getWaypoints().isEmpty()) {
                            sendCurrentWaypointGoal();
                        } else {
                            isPatrolling = false;
                            isNavigating = false;
                            Toast.makeText(this, "🎉 모든 경유지 순찰 완료!", Toast.LENGTH_SHORT).show();
                            tvSelectedGoalPos.setText("모든 경유지 순찰 완료");
                        }
                    }
                }
            }
        });
    }

    @Override
    public void onBatteryStateReceived(float percentage, float voltage, boolean isCharging) {
        runOnUiThread(() -> {
            String statusStr = String.format(Locale.getDefault(), "%s %.0f%% (%.1fV)", isCharging ? "⚡" : "🔋", percentage, voltage);
            tvBatteryBadge.setText(statusStr);
            if (percentage > 50) tvBatteryBadge.setTextColor(Color.GREEN);
            else if (percentage > 20) tvBatteryBadge.setTextColor(Color.YELLOW);
            else tvBatteryBadge.setTextColor(Color.RED);
        });
    }

    @Override
    public void onCustomTopicReceived(String topic, String data) {
        runOnUiThread(() -> {
            if (topic.equals("/rqt_graph_data") && rqtGraphView != null) {
                rqtGraphView.updateGraphFromJson(data);
                return;
            }

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
    public void onServiceResponseReceived(String serviceName, String result) {
        runOnUiThread(() -> {
            String timestamp = timeFormat.format(new Date());
            tvLogConsole.append(String.format(Locale.getDefault(), "\n[%s] %s ➔ %s\n", timestamp, serviceName, result));
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