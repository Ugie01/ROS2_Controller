#!/usr/bin/env python3
import rclpy
import json
import subprocess
import os
from rclpy.node import Node
from std_msgs.msg import String

class ROS2AppBridge(Node):
    def __init__(self):
        super().__init__('ros2_app_bridge')
        self.create_subscription(String, '/param_set_cmd', self.param_callback, 10)
        self.create_subscription(String, '/save_map_cmd', self.save_map_callback, 10)
        self.create_subscription(String, '/request_rqt_graph', self.rqt_graph_callback, 10)
        
        self.graph_pub = self.create_publisher(String, '/rqt_graph_data', 10)
        self.cwd = os.getcwd()
        self.get_logger().info(f'🚀 [rqt_graph 그룹화 브릿지 실행 중] (Domain: {os.environ.get("ROS_DOMAIN_ID", "0")})')

    def param_callback(self, msg):
        try:
            data = json.loads(msg.data)
            node_name = data.get("node")
            param_name = data.get("param")
            param_val = str(data.get("value"))

            if node_name == '/global_costmap':
                node_name = '/global_costmap/global_costmap'
                if param_name.startswith('global_costmap.'):
                    param_name = param_name.replace('global_costmap.', '', 1)
            elif node_name == '/local_costmap':
                node_name = '/local_costmap/local_costmap'
                if param_name.startswith('local_costmap.'):
                    param_name = param_name.replace('local_costmap.', '', 1)

            self.get_logger().info(f'⚙️ [파라미터 적용] {node_name} ➔ {param_name} = {param_val}')
            cmd = ['ros2', 'param', 'set', node_name, param_name, param_val]
            res = subprocess.run(cmd, capture_output=True, text=True, env=os.environ.copy())

            if res.returncode == 0:
                self.get_logger().info(f'🎉 파라미터 적용 성공')
            else:
                self.get_logger().error(f'❌ 적용 실패: {res.stderr.strip()}')
        except Exception as e:
            self.get_logger().error(f'❌ 파라미터 오류: {str(e)}')

    def save_map_callback(self, msg):
        map_name = msg.data.strip() or 'my_map'
        save_path = os.path.join(self.cwd, map_name)
        self.get_logger().info(f'💾 [맵 저장] 경로: {save_path}')
        res = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', save_path], capture_output=True, text=True, env=os.environ.copy())
        if res.returncode == 0:
            self.get_logger().info(f'🎉 맵 저장 성공! ({save_path}.yaml, .pgm)')
        else:
            self.get_logger().error(f'❌ 맵 저장 실패: {res.stderr.strip()}')

    def rqt_graph_callback(self, msg):
        """PC rqt_graph와 동일한 네임스페이스 그룹핑 및 계층 구조 생성"""
        try:
            self.get_logger().info('📊 rqt_graph 네임스페이스 그룹화 분석 시작...')
            
            node_tuples = self.get_node_names_and_namespaces()
            nodes_dict = {}
            nodes_list = []
            links_list = []
            groups_dict = {}

            # 1. 노드 등록
            for name, ns in node_tuples:
                full_name = f"{ns.rstrip('/')}/{name}" if ns != '/' else f"/{name}"
                if 'ros2_app_bridge' in full_name or 'transform_listener' in full_name or '_rclpy_' in full_name:
                    continue
                idx = len(nodes_list)
                nodes_dict[full_name] = idx
                group_name = ns if ns != '/' else ''
                nodes_list.append({"name": full_name, "isTopic": False, "group": group_name})

            # 2. 토픽 및 링크 수집 (지저분한 액션 내부 토픽 5종 세트 자동 축약)
            topic_dict = {}
            
            for name, ns in node_tuples:
                full_name = f"{ns.rstrip('/')}/{name}" if ns != '/' else f"/{name}"
                if full_name not in nodes_dict: continue
                n_idx = nodes_dict[full_name]

                # 발행 토픽
                try:
                    pub_topics = self.get_publisher_names_and_types_by_node(name, ns)
                    for t_name, _ in pub_topics:
                        # 디버그/임시 토픽 필터링
                        if any(x in t_name for x in ['parameter_events', 'rosout', 'rqt_graph', 'save_map', 'param_set']):
                            continue
                        
                        # 액션 내부 서브토픽 축약 (_action/status, _action/feedback -> _action)
                        display_topic = t_name
                        if '/_action/' in display_topic:
                            display_topic = display_topic.split('/_action/')[0] + '/_action'

                        if display_topic not in topic_dict:
                            t_idx = len(nodes_list)
                            topic_dict[display_topic] = t_idx
                            nodes_list.append({"name": display_topic, "isTopic": True, "group": ""})
                        else:
                            t_idx = topic_dict[display_topic]
                        
                        if not any(l["from"] == n_idx and l["to"] == t_idx for l in links_list):
                            links_list.append({"from": n_idx, "to": t_idx})
                except Exception:
                    pass

                # 구독 토픽
                try:
                    sub_topics = self.get_subscriber_names_and_types_by_node(name, ns)
                    for t_name, _ in sub_topics:
                        if any(x in t_name for x in ['parameter_events', 'rosout', 'rqt_graph', 'save_map', 'param_set']):
                            continue
                        
                        display_topic = t_name
                        if '/_action/' in display_topic:
                            display_topic = display_topic.split('/_action/')[0] + '/_action'

                        if display_topic not in topic_dict:
                            t_idx = len(nodes_list)
                            topic_dict[display_topic] = t_idx
                            nodes_list.append({"name": display_topic, "isTopic": True, "group": ""})
                        else:
                            t_idx = topic_dict[display_topic]
                        
                        if not any(l["from"] == t_idx and l["to"] == n_idx for l in links_list):
                            links_list.append({"from": t_idx, "to": n_idx})
                except Exception:
                    pass

            # 3. 4단 계층형 레이아웃 배치 (Sensors -> Map/TF -> Nav/Plan -> Actuators)
            col0_y, col1_y, col2_y, col3_y = 90, 90, 90, 90
            for i, item in enumerate(nodes_list):
                name = item["name"]
                if item["isTopic"]:
                    if any(x in name for x in ['scan', 'image', 'camera', 'odom']):
                        item["x"] = 380; item["y"] = col1_y; col1_y += 75
                    elif any(x in name for x in ['cmd_vel', 'speed']):
                        item["x"] = 1140; item["y"] = col3_y; col3_y += 75
                    else:
                        item["x"] = 760; item["y"] = col2_y; col2_y += 75
                else:
                    if any(x in name for x in ['driver', 'lidar', 'camera', 'joint']):
                        item["x"] = 40; item["y"] = col0_y; col0_y += 105
                    elif any(x in name for x in ['controller', 'base', 'diff']):
                        item["x"] = 1420; item["y"] = col3_y; col3_y += 105
                    elif any(x in name for x in ['slam', 'amcl', 'map']):
                        item["x"] = 40; item["y"] = col0_y; col0_y += 105
                    else:
                        item["x"] = 1040; item["y"] = col2_y; col2_y += 105

            graph_data = {"nodes": nodes_list, "links": links_list}
            reply_msg = String()
            reply_msg.data = json.dumps(graph_data)
            self.graph_pub.publish(reply_msg)
            self.get_logger().info(f'✅ rqt_graph 정리 완료: 노드 {len(nodes_dict)}개, 토픽 {len(topic_dict)}개, 링크 {len(links_list)}개')
        except Exception as e:
            self.get_logger().error(f'❌ rqt_graph 분석 오류: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ROS2AppBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()