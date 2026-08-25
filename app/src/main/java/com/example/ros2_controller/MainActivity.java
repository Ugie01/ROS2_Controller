package com.example.ros2_controller;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

public class MainActivity extends AppCompatActivity implements ROS2NativeManager.ROS2DataListener {
    private static final String TAG = "ROS2_DEBUG_Main";

    private EditText etDomainId;
    private Button btnNodeToggle, btnEmergencyStop;
    private TextView tvStatus, tvVelocity;
    private ImageView ivCamera;
    private LaserScanView laserScanView;
    private JoystickView joystickView;

    private ROS2NativeManager ros2Manager;

    private float linearX = 0f;
    private float angularZ = 0f;
    private final Handler timerHandler = new Handler(Looper.getMainLooper());
    private final float MAX_LINEAR_VEL = 0.5f;
    private final float MAX_ANGULAR_VEL = 1.0f;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i(TAG, "onCreate 시작");
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main);

        ViewCompat.setOnApplyWindowInsetsListener(findViewById(R.id.main), (v, insets) -> {
            Insets systemBars = insets.getInsets(WindowInsetsCompat.Type.systemBars());
            v.setPadding(systemBars.left, systemBars.top, systemBars.right, systemBars.bottom);
            return insets;
        });

        etDomainId = findViewById(R.id.etDomainId);
        btnNodeToggle = findViewById(R.id.btnNodeToggle);
        btnEmergencyStop = findViewById(R.id.btnEmergencyStop);
        tvStatus = findViewById(R.id.tvStatus);
        tvVelocity = findViewById(R.id.tvVelocity);
        ivCamera = findViewById(R.id.ivCamera);
        laserScanView = findViewById(R.id.laserScanView);
        joystickView = findViewById(R.id.joystickView);

        ros2Manager = new ROS2NativeManager(this, this);

        btnNodeToggle.setOnClickListener(v -> {
            if (!ros2Manager.isRunning()) {
                String domainStr = etDomainId.getText().toString().trim();
                int domainId = domainStr.isEmpty() ? 0 : Integer.parseInt(domainStr);
                Log.d(TAG, "노드 시작 버튼 클릭됨 -> 대상 도메인: " + domainId);
                ros2Manager.startNode(domainId);
            } else {
                Log.d(TAG, "노드 중지 버튼 클릭됨");
                ros2Manager.stopNode();
            }
        });

        joystickView.setJoystickListener((xPercent, yPercent) -> {
            linearX = yPercent * MAX_LINEAR_VEL;
            angularZ = -xPercent * MAX_ANGULAR_VEL;
            tvVelocity.setText(String.format("Linear: %.2f m/s | Angular: %.2f rad/s", linearX, angularZ));

            if (xPercent == 0f && yPercent == 0f && ros2Manager.isRunning()) {
                Log.d(TAG, "조이스틱 중립 감지 -> 정지 명령(0,0) 전송");
                ros2Manager.publishCmdVel(0f, 0f);
            }
        });

        btnEmergencyStop.setOnClickListener(v -> {
            Log.w(TAG, "긴급 정지(E-STOP) 버튼 클릭됨");
            linearX = 0f;
            angularZ = 0f;
            ros2Manager.publishCmdVel(0f, 0f);
            tvVelocity.setText("Linear: 0.00 m/s | Angular: 0.00 rad/s (E-STOP)");
        });

        startPublishingLoop();
    }

    private void startPublishingLoop() {
        timerHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (ros2Manager.isRunning() && (linearX != 0 || angularZ != 0)) {
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
    public void onStatusChanged(String status, boolean isRunning) {
        Log.d(TAG, "UI 상태 갱신: " + status + " (isRunning: " + isRunning + ")");
        runOnUiThread(() -> {
            tvStatus.setText(status);
            if (isRunning) {
                tvStatus.setTextColor(Color.GREEN);
                btnNodeToggle.setText("DDS 노드 중지");
                btnNodeToggle.setBackgroundColor(Color.RED);
            } else {
                tvStatus.setTextColor(Color.RED);
                btnNodeToggle.setText("DDS 노드 시작");
                btnNodeToggle.setBackgroundColor(Color.parseColor("#4CAF50"));
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i(TAG, "onDestroy 호출됨");
        ros2Manager.stopNode();
        timerHandler.removeCallbacksAndMessages(null);
    }
}