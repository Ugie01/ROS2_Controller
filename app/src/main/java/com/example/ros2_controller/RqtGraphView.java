package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RqtGraphView extends View {

    public static class GraphNode {
        public String name;
        public boolean isTopic;
        public String group;
        public float x, y;
        public float width, height;

        public GraphNode(String name, boolean isTopic, String group, float x, float y) {
            this.name = name;
            this.isTopic = isTopic;
            this.group = group;
            this.x = x;
            this.y = y;
            this.width = Math.max(160f, name.length() * 12.5f + 36f);
            this.height = isTopic ? 42f : 58f;
        }
    }

    public static class GraphLink {
        public int fromIdx;
        public int toIdx;

        public GraphLink(int fromIdx, int toIdx) {
            this.fromIdx = fromIdx;
            this.toIdx = toIdx;
        }
    }

    private final List<GraphNode> nodes = new ArrayList<>();
    private final List<GraphLink> links = new ArrayList<>();

    // 페인트
    private Paint nodeFillPaint, nodeStrokePaint;
    private Paint topicFillPaint, topicStrokePaint;
    private Paint nodeTextPaint, topicTextPaint;
    private Paint curveLinePaint, arrowPaint;
    private Paint groupStrokePaint, groupTextPaint, groupFillPaint;
    private Paint emptyTextPaint, emptySubTextPaint;

    private float scaleFactor = 0.85f;
    private float posX = 50f, posY = 60f;
    private float lastTouchX, lastTouchY;
    private ScaleGestureDetector scaleDetector;

    public RqtGraphView(Context context, AttributeSet attrs) {
        super(context, attrs);
        initPaints();
    }

    private void initPaints() {
        scaleDetector = new ScaleGestureDetector(getContext(), new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                scaleFactor *= detector.getScaleFactor();
                scaleFactor = Math.max(0.2f, Math.min(scaleFactor, 2.5f));
                invalidate();
                return true;
            }
        });

        // 1. 노드: 파란색 타원
        nodeFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        nodeFillPaint.setColor(Color.parseColor("#1A237E"));
        nodeFillPaint.setStyle(Paint.Style.FILL);

        nodeStrokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        nodeStrokePaint.setColor(Color.parseColor("#448AFF"));
        nodeStrokePaint.setStyle(Paint.Style.STROKE);
        nodeStrokePaint.setStrokeWidth(3f);

        // 2. 토픽: 초록색 직사각형 박스
        topicFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        topicFillPaint.setColor(Color.parseColor("#1B5E20"));
        topicFillPaint.setStyle(Paint.Style.FILL);

        topicStrokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        topicStrokePaint.setColor(Color.parseColor("#00E676"));
        topicStrokePaint.setStyle(Paint.Style.STROKE);
        topicStrokePaint.setStrokeWidth(2.5f);

        // 3. [네임스페이스 그룹 박스 스타일]
        groupFillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        groupFillPaint.setColor(Color.parseColor("#151E1E1E"));
        groupFillPaint.setStyle(Paint.Style.FILL);

        groupStrokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        groupStrokePaint.setColor(Color.parseColor("#555555"));
        groupStrokePaint.setStyle(Paint.Style.STROKE);
        groupStrokePaint.setStrokeWidth(2f);
        groupStrokePaint.setPathEffect(new DashPathEffect(new float[]{10, 8}, 0));

        groupTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        groupTextPaint.setColor(Color.parseColor("#888888"));
        groupTextPaint.setTextSize(18f);
        groupTextPaint.setFakeBoldText(true);

        // 4. 텍스트
        nodeTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        nodeTextPaint.setColor(Color.WHITE);
        nodeTextPaint.setTextSize(20f);
        nodeTextPaint.setTextAlign(Paint.Align.CENTER);
        nodeTextPaint.setFakeBoldText(true);

        topicTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        topicTextPaint.setColor(Color.parseColor("#E0E0E0"));
        topicTextPaint.setTextSize(18f);
        topicTextPaint.setTextAlign(Paint.Align.CENTER);

        // 5. 연결선 및 화살표
        curveLinePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        curveLinePaint.setColor(Color.parseColor("#78909C"));
        curveLinePaint.setStrokeWidth(3.5f);
        curveLinePaint.setStyle(Paint.Style.STROKE);

        arrowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        arrowPaint.setColor(Color.parseColor("#00E676"));
        arrowPaint.setStyle(Paint.Style.FILL);

        emptyTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        emptyTextPaint.setColor(Color.parseColor("#888888"));
        emptyTextPaint.setTextSize(26f);
        emptyTextPaint.setTextAlign(Paint.Align.CENTER);

        emptySubTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        emptySubTextPaint.setColor(Color.parseColor("#555555"));
        emptySubTextPaint.setTextSize(20f);
        emptySubTextPaint.setTextAlign(Paint.Align.CENTER);

        nodes.clear();
        links.clear();
    }

    public synchronized void resetView() {
        this.posX = 50f;
        this.posY = 60f;
        this.scaleFactor = 0.85f;
        postInvalidate();
    }

    public synchronized void updateGraphFromJson(String jsonStr) {
        try {
            JSONObject root = new JSONObject(jsonStr);
            JSONArray nodesArray = root.getJSONArray("nodes");
            JSONArray linksArray = root.getJSONArray("links");

            nodes.clear();
            links.clear();

            for (int i = 0; i < nodesArray.length(); i++) {
                JSONObject obj = nodesArray.getJSONObject(i);
                String name = obj.getString("name");
                boolean isTopic = obj.optBoolean("isTopic", false);
                String group = obj.optString("group", "");
                float x = (float) obj.optDouble("x", 60);
                float y = (float) obj.optDouble("y", 80);
                nodes.add(new GraphNode(name, isTopic, group, x, y));
            }

            for (int i = 0; i < linksArray.length(); i++) {
                JSONObject linkObj = linksArray.getJSONObject(i);
                int from = linkObj.getInt("from");
                int to = linkObj.getInt("to");
                if (from < nodes.size() && to < nodes.size()) {
                    links.add(new GraphLink(from, to));
                }
            }
            resetView();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (nodes.isEmpty()) {
            canvas.drawText("📊 실행 중인 ROS 2 노드가 없습니다.", getWidth() / 2f, getHeight() / 2f - 20, emptyTextPaint);
            canvas.drawText("PC에서 노드/로봇 실행 후 [🔄 새로고침]을 누르세요.", getWidth() / 2f, getHeight() / 2f + 30, emptySubTextPaint);
            return;
        }

        canvas.save();
        canvas.translate(posX, posY);
        canvas.scale(scaleFactor, scaleFactor);

        // 1. [네임스페이스 그룹 박스 그리기]
        Map<String, RectF> groupBounds = new HashMap<>();
        for (GraphNode node : nodes) {
            if (!node.group.isEmpty()) {
                RectF r = groupBounds.get(node.group);
                if (r == null) {
                    r = new RectF(node.x, node.y, node.x + node.width, node.y + node.height);
                    groupBounds.put(node.group, r);
                } else {
                    r.union(node.x, node.y, node.x + node.width, node.y + node.height);
                }
            }
        }
        for (Map.Entry<String, RectF> entry : groupBounds.entrySet()) {
            RectF r = entry.getValue();
            r.inset(-16f, -22f);
            canvas.drawRoundRect(r, 12f, 12f, groupFillPaint);
            canvas.drawRoundRect(r, 12f, 12f, groupStrokePaint);
            canvas.drawText(entry.getKey(), r.left + 12f, r.top + 16f, groupTextPaint);
        }

        // 2. [부드러운 S자 베지어 곡선 연결선 & 화살표]
        for (GraphLink link : links) {
            if (link.fromIdx >= nodes.size() || link.toIdx >= nodes.size()) continue;
            GraphNode from = nodes.get(link.fromIdx);
            GraphNode to = nodes.get(link.toIdx);

            float startX = from.x + from.width;
            float startY = from.y + from.height / 2f;
            float endX = to.x;
            float endY = to.y + to.height / 2f;

            if (startX > endX) {
                startX = from.x;
                endX = to.x + to.width;
            }

            float ctrlX1 = startX + Math.abs(endX - startX) * 0.5f;
            float ctrlY1 = startY;
            float ctrlX2 = endX - Math.abs(endX - startX) * 0.5f;
            float ctrlY2 = endY;

            Path bezierPath = new Path();
            bezierPath.moveTo(startX, startY);
            bezierPath.cubicTo(ctrlX1, ctrlY1, ctrlX2, ctrlY2, endX, endY);
            canvas.drawPath(bezierPath, curveLinePaint);

            drawArrow(canvas, ctrlX2, ctrlY2, endX, endY);
        }

        // 3. [노드(타원) 및 토픽(직사각형) 그리기]
        for (GraphNode node : nodes) {
            RectF rect = new RectF(node.x, node.y, node.x + node.width, node.y + node.height);

            if (node.isTopic) {
                canvas.drawRoundRect(rect, 6f, 6f, topicFillPaint);
                canvas.drawRoundRect(rect, 6f, 6f, topicStrokePaint);
                canvas.drawText(node.name, node.x + node.width / 2f, node.y + node.height / 2f + 6f, topicTextPaint);
            } else {
                canvas.drawOval(rect, nodeFillPaint);
                canvas.drawOval(rect, nodeStrokePaint);
                canvas.drawText(node.name, node.x + node.width / 2f, node.y + node.height / 2f + 7f, nodeTextPaint);
            }
        }

        canvas.restore();
    }

    private void drawArrow(Canvas canvas, float fromX, float fromY, float toX, float toY) {
        float angle = (float) Math.atan2(toY - fromY, toX - fromX);
        float arrowSize = 14f;

        Path path = new Path();
        path.moveTo(toX, toY);
        path.lineTo(toX - arrowSize * (float) Math.cos(angle - Math.PI / 6), toY - arrowSize * (float) Math.sin(angle - Math.PI / 6));
        path.lineTo(toX - arrowSize * (float) Math.cos(angle + Math.PI / 6), toY - arrowSize * (float) Math.sin(angle + Math.PI / 6));
        path.close();

        canvas.drawPath(path, arrowPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        scaleDetector.onTouchEvent(event);
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                lastTouchX = event.getX();
                lastTouchY = event.getY();
                break;
            case MotionEvent.ACTION_MOVE:
                if (!scaleDetector.isInProgress()) {
                    float dx = event.getX() - lastTouchX;
                    float dy = event.getY() - lastTouchY;
                    posX += dx;
                    posY += dy;
                    posX = Math.max(-3000f, Math.min(posX, 3000f));
                    posY = Math.max(-3000f, Math.min(posY, 3000f));
                    invalidate();
                }
                lastTouchX = event.getX();
                lastTouchY = event.getY();
                break;
        }
        return true;
    }
}