package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

public class MapView extends View {

    // 맵 모드 상수 정의
    public static final int MODE_SINGLE_GOAL = 0;
    public static final int MODE_WAYPOINTS = 1;
    public static final int MODE_INITIAL_POSE = 2; // rviz2 2D Pose Estimate 모드

    private int currentMode = MODE_SINGLE_GOAL;

    // 1. 점유격자 맵 버퍼
    private Bitmap cachedMapBitmap;
    private int[] cachedPixels;
    private int prevWidth = 0, prevHeight = 0;
    private float resolution = 0.05f;
    private float originX = 0f, originY = 0f;
    private int mapWidth = 0, mapHeight = 0;

    // 2. 로봇 현재 위치 & 방향 (amcl_pose / odom)
    private float robotWorldX = 0f, robotWorldY = 0f, robotYaw = 0f;
    private boolean hasRobotPose = false;

    // 3. 단일 목표 (Goal) & 도착 각도 (Yaw)
    private float goalWorldX = 0f, goalWorldY = 0f, goalYaw = 0f;
    private float goalPixelX = -1f, goalPixelY = -1f;
    private boolean hasGoal = false;
    private boolean isDraggingHeading = false;

    // 4. 2D Pose Estimate (초기 위치 및 자세)
    private float initPoseWorldX = 0f, initPoseWorldY = 0f, initPoseYaw = 0f;
    private float initPosePixelX = -1f, initPosePixelY = -1f;
    private boolean hasInitPose = false;

    // 5. 다중 경유지 (Waypoints) 리스트
    public static class Waypoint {
        public float x, y, yaw;
        public float px, py;
        public Waypoint(float x, float y, float yaw, float px, float py) {
            this.x = x; this.y = y; this.yaw = yaw;
            this.px = px; this.py = py;
        }
    }
    private final List<Waypoint> waypointList = new ArrayList<>();

    // 페인트 객체
    private Paint goalPaint, goalHeadingPaint, robotPaint, robotHeadingPaint;
    private Paint initPosePaint, initPoseHeadingPaint;
    private Paint textPaint, waypointPaint, pathLinePaint;
    private final Matrix matrix = new Matrix();

    public interface OnMapEventListener {
        void onSingleGoalSelected(float worldX, float worldY, float yaw);
        void onInitialPoseSelected(float worldX, float worldY, float yaw);
        void onWaypointsUpdated(int count);
    }
    private OnMapEventListener mapEventListener;

    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initPaints();
    }

    private void initPaints() {
        // 단일 목표 마커 (Red)
        goalPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        goalPaint.setColor(Color.parseColor("#FF1744"));
        goalPaint.setStyle(Paint.Style.FILL);

        goalHeadingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        goalHeadingPaint.setColor(Color.parseColor("#FF5252"));
        goalHeadingPaint.setStrokeWidth(4f);

        // 2D Pose Estimate 마커 (Green / Yellow)
        initPosePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        initPosePaint.setColor(Color.parseColor("#00E676"));
        initPosePaint.setStyle(Paint.Style.FILL);

        initPoseHeadingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        initPoseHeadingPaint.setColor(Color.parseColor("#B9F6CA"));
        initPoseHeadingPaint.setStrokeWidth(5f);

        // 로봇 본체 마커 (Cyan)
        robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        robotPaint.setColor(Color.parseColor("#00E5FF"));
        robotPaint.setStyle(Paint.Style.FILL);

        robotHeadingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        robotHeadingPaint.setColor(Color.parseColor("#76FF03"));
        robotHeadingPaint.setStrokeWidth(4f);

        // 경유지 마커 (Yellow)
        waypointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        waypointPaint.setColor(Color.parseColor("#FFD600"));
        waypointPaint.setStyle(Paint.Style.FILL);

        pathLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pathLinePaint.setColor(Color.parseColor("#FFD600"));
        pathLinePaint.setStrokeWidth(3f);
        pathLinePaint.setStyle(Paint.Style.STROKE);
        pathLinePaint.setPathEffect(new DashPathEffect(new float[]{10, 10}, 0));

        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(24f);
        textPaint.setFakeBoldText(true);
    }

    public void setOnMapEventListener(OnMapEventListener listener) {
        this.mapEventListener = listener;
    }

    public void setMode(int mode) {
        this.currentMode = mode;
        clearAllMarkers();
    }

    public void clearAllMarkers() {
        hasGoal = false;
        hasInitPose = false;
        isDraggingHeading = false;
        waypointList.clear();
        if (mapEventListener != null) mapEventListener.onWaypointsUpdated(0);
        postInvalidate();
    }

    public synchronized void removeFirstWaypoint() {
        if (!waypointList.isEmpty()) {
            waypointList.remove(0);
            if (mapEventListener != null) {
                mapEventListener.onWaypointsUpdated(waypointList.size());
            }
            postInvalidate();
        }
    }

    public List<Waypoint> getWaypoints() {
        return waypointList;
    }

    public synchronized void updateMap(byte[] data, int width, int height, float resolution, float originX, float originY) {
        this.mapWidth = width;
        this.mapHeight = height;
        this.resolution = resolution;
        this.originX = originX;
        this.originY = originY;

        int totalPixels = width * height;
        if (cachedPixels == null || prevWidth != width || prevHeight != height) {
            cachedPixels = new int[totalPixels];
            cachedMapBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
            prevWidth = width;
            prevHeight = height;
        }

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int srcIdx = (height - 1 - y) * width + x;
                if (srcIdx < data.length) {
                    byte val = data[srcIdx];
                    int color;
                    if (val == -1) color = 0xFF2E2E2E;
                    else if (val == 0) color = 0xFFE0E0E0;
                    else if (val == 100) color = 0xFF000000;
                    else color = 0xFFFF5722;

                    cachedPixels[y * width + x] = color;
                }
            }
        }
        cachedMapBitmap.setPixels(cachedPixels, 0, width, 0, 0, width, height);
        postInvalidate();
    }

    public synchronized void updateRobotPose(float worldX, float worldY, float yaw) {
        this.robotWorldX = worldX;
        this.robotWorldY = worldY;
        this.robotYaw = yaw;
        this.hasRobotPose = true;
        postInvalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (cachedMapBitmap != null) {
            float scaleX = (float) getWidth() / cachedMapBitmap.getWidth();
            float scaleY = (float) getHeight() / cachedMapBitmap.getHeight();
            float scale = Math.min(scaleX, scaleY);

            float dx = (getWidth() - cachedMapBitmap.getWidth() * scale) / 2f;
            float dy = (getHeight() - cachedMapBitmap.getHeight() * scale) / 2f;

            matrix.reset();
            matrix.postScale(scale, scale);
            matrix.postTranslate(dx, dy);
            canvas.drawBitmap(cachedMapBitmap, matrix, null);

            // 1. 다중 경유지 (Waypoints) 및 연결 경로 점선 그리기
            if (!waypointList.isEmpty()) {
                float lastX = -1, lastY = -1;
                for (int i = 0; i < waypointList.size(); i++) {
                    Waypoint wp = waypointList.get(i);
                    float sx = dx + wp.px * scale;
                    float sy = dy + wp.py * scale;

                    if (lastX != -1) {
                        canvas.drawLine(lastX, lastY, sx, sy, pathLinePaint);
                    }
                    lastX = sx;
                    lastY = sy;

                    canvas.drawCircle(sx, sy, 13f, waypointPaint);
                    canvas.drawText(String.valueOf(i + 1), sx - 6, sy + 8, textPaint);
                }
            }

            // 2. 단일 목표 (Goal) 마커 및 도착 각도 화살표
            if (hasGoal && goalPixelX >= 0 && goalPixelY >= 0) {
                float sx = dx + goalPixelX * scale;
                float sy = dy + goalPixelY * scale;

                canvas.drawCircle(sx, sy, 12f, goalPaint);
                float hx = sx + (float) (35f * Math.cos(goalYaw));
                float hy = sy - (float) (35f * Math.sin(goalYaw));
                canvas.drawLine(sx, sy, hx, hy, goalHeadingPaint);
                canvas.drawCircle(hx, hy, 4f, goalHeadingPaint);
            }

            // 3. 2D Pose Estimate (초기 위치 및 자세 마커)
            if (hasInitPose && initPosePixelX >= 0 && initPosePixelY >= 0) {
                float sx = dx + initPosePixelX * scale;
                float sy = dy + initPosePixelY * scale;

                canvas.drawCircle(sx, sy, 14f, initPosePaint);
                float hx = sx + (float) (40f * Math.cos(initPoseYaw));
                float hy = sy - (float) (40f * Math.sin(initPoseYaw));
                canvas.drawLine(sx, sy, hx, hy, initPoseHeadingPaint);
                canvas.drawCircle(hx, hy, 5f, initPoseHeadingPaint);
            }

            // 4. 로봇 현재 위치 & 방향 (amcl_pose / odom)
            if (hasRobotPose && resolution > 0) {
                float rx = (robotWorldX - originX) / resolution;
                float ry = mapHeight - 1 - (robotWorldY - originY) / resolution;
                float sx = dx + rx * scale;
                float sy = dy + ry * scale;

                canvas.drawCircle(sx, sy, 12f, robotPaint);
                float hx = sx + (float) (28f * Math.cos(robotYaw));
                float hy = sy - (float) (28f * Math.sin(robotYaw));
                canvas.drawLine(sx, sy, hx, hy, robotHeadingPaint);
                canvas.drawCircle(hx, hy, 4f, robotHeadingPaint);
            }
        } else {
            canvas.drawText("맵 데이터 대기 중 (/map)...", 40, getHeight() / 2f, textPaint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (cachedMapBitmap == null) return super.onTouchEvent(event);

        float scaleX = (float) getWidth() / cachedMapBitmap.getWidth();
        float scaleY = (float) getHeight() / cachedMapBitmap.getHeight();
        float scale = Math.min(scaleX, scaleY);
        float dx = (getWidth() - cachedMapBitmap.getWidth() * scale) / 2f;
        float dy = (getHeight() - cachedMapBitmap.getHeight() * scale) / 2f;

        float touchX = (event.getX() - dx) / scale;
        float touchY = (event.getY() - dy) / scale;

        if (touchX < 0 || touchX >= mapWidth || touchY < 0 || touchY >= mapHeight) {
            return super.onTouchEvent(event);
        }

        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                if (currentMode == MODE_WAYPOINTS) {
                    float gridY = mapHeight - 1 - touchY;
                    float wx = originX + touchX * resolution;
                    float wy = originY + gridY * resolution;
                    waypointList.add(new Waypoint(wx, wy, 0f, touchX, touchY));
                    if (mapEventListener != null) {
                        mapEventListener.onWaypointsUpdated(waypointList.size());
                    }
                    invalidate();
                } else if (currentMode == MODE_INITIAL_POSE) {
                    initPosePixelX = touchX;
                    initPosePixelY = touchY;
                    float gridY = mapHeight - 1 - touchY;
                    initPoseWorldX = originX + touchX * resolution;
                    initPoseWorldY = originY + gridY * resolution;
                    initPoseYaw = 0f;
                    hasInitPose = true;
                    isDraggingHeading = true;
                    invalidate();
                } else {
                    goalPixelX = touchX;
                    goalPixelY = touchY;
                    float gridY = mapHeight - 1 - touchY;
                    goalWorldX = originX + touchX * resolution;
                    goalWorldY = originY + gridY * resolution;
                    goalYaw = 0f;
                    hasGoal = true;
                    isDraggingHeading = true;
                    invalidate();
                }
                return true;

            case MotionEvent.ACTION_MOVE:
                if (isDraggingHeading) {
                    if (currentMode == MODE_INITIAL_POSE) {
                        float dX = touchX - initPosePixelX;
                        float dY = -(touchY - initPosePixelY);
                        if (Math.hypot(dX, dY) > 5) {
                            initPoseYaw = (float) Math.atan2(dY, dX);
                            invalidate();
                        }
                    } else if (currentMode == MODE_SINGLE_GOAL) {
                        float dX = touchX - goalPixelX;
                        float dY = -(touchY - goalPixelY);
                        if (Math.hypot(dX, dY) > 5) {
                            goalYaw = (float) Math.atan2(dY, dX);
                            invalidate();
                        }
                    }
                }
                return true;

            case MotionEvent.ACTION_UP:
                if (isDraggingHeading) {
                    isDraggingHeading = false;
                    if (currentMode == MODE_INITIAL_POSE && mapEventListener != null) {
                        mapEventListener.onInitialPoseSelected(initPoseWorldX, initPoseWorldY, initPoseYaw);
                    } else if (currentMode == MODE_SINGLE_GOAL && mapEventListener != null) {
                        mapEventListener.onSingleGoalSelected(goalWorldX, goalWorldY, goalYaw);
                    }
                    invalidate();
                }
                return true;
        }
        return super.onTouchEvent(event);
    }
}