import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import os
import re

DB_PATH = os.path.join(os.path.dirname(__file__), 'database.db')

def update_slot_status(parsed_data):
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # 테이블 없으면 생성
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS yolo_slot_status (
            slot_id INTEGER PRIMARY KEY,
            object_name TEXT,
            quantity INTEGER,
            status TEXT,
            timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    ''')

    for slot_id, (content, status) in parsed_data.items():
        if content:  # 내용이 있는 경우
            parts = content.split()
            object_name = parts[0] if len(parts) > 0 else ''
            try:
                quantity = int(parts[1].replace('개', '')) if len(parts) > 1 else 0
            except ValueError:
                quantity = 0
        else:
            object_name = ''
            quantity = 0

        cursor.execute('''
            INSERT OR REPLACE INTO yolo_slot_status (slot_id, object_name, quantity, status, timestamp)
            VALUES (?, ?, ?, ?, CURRENT_TIMESTAMP)
        ''', (slot_id, object_name, quantity, status))

    conn.commit()
    conn.close()


def parse_yolo_result_text(text):
    """
    ROS 토픽 메시지 문자열을 파싱해서 dict 반환
    {
        0: ('box 2개', '정상'),
        1: ('tape 1개', '정상'),
        ...
    }
    """
    parsed = {}
    # 예시 메시지 형식:
    # Slot 0: box 2개 [정상]
    lines = text.strip().split('\n')
    pattern = re.compile(r'Slot (\d+): (.+) \[(.+)\]')
    for line in lines:
        m = pattern.match(line.strip())
        if m:
            slot_id = int(m.group(1))
            content = m.group(2).strip()
            status = m.group(3).strip()
            parsed[slot_id] = (content, status)
    return parsed

class YoloResultLogger(Node):
    def __init__(self):
        super().__init__('yolo_result_logger')

        self.latest_result = "초기화"
        self.subscription = self.create_subscription(
            String,
            'yolo_slot_result',
            self.result_callback,
            10
        )
        self.get_logger().info('YOLO 토픽 구독 시작')

        # 10초마다 DB 저장 타이머
        self.timer = self.create_timer(10.0, self.save_to_db)

    def result_callback(self, msg):
        self.latest_result = msg.data
        self.get_logger().info(f'결과 수신: {self.latest_result[:50]}...')

    def save_to_db(self):
        parsed = parse_yolo_result_text(self.latest_result)
        if parsed:
            update_slot_status(parsed)
            self.get_logger().info('10초마다 DB 업데이트 완료')
        else:
            self.get_logger().warn('파싱 실패 또는 빈 결과, DB 업데이트 안함')

def main(args=None):
    rclpy.init(args=args)
    node = YoloResultLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
