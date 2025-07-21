from flask import Flask, render_template, request, redirect, url_for
from flask import jsonify
import sqlite3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import threading
import time # 로깅/디버깅 목적
from yolo_result_logger import YoloResultLogger

app = Flask(__name__)

# DB 경로 설정
DB_PATH = os.path.join(os.path.dirname(__file__), 'database', 'database.db')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/login')
def login():
    return render_template('login.html')

@app.route('/order')
def order():
    return render_template('order.html')

@app.route('/admin')
def admin():
    return render_template('admin.html')

@app.route('/login_check', methods=['POST'])
def login_check():
    user_id = request.form['id']
    password = request.form['pw']

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT * FROM users WHERE id = ? AND password = ?", (user_id, password))
    user = cursor.fetchone()
    conn.close()

    if user:
        return user_id  
    else:
        return "fail"


@app.route('/submit_order', methods=['POST'])
def submit_order():
    name = request.form['name']
    address = request.form['address']
    phone = request.form['phone']

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # orders 테이블이 없을 경우 대비
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            address TEXT,
            phone TEXT
        )
    ''')

    cursor.execute("INSERT INTO orders (name, address, phone) VALUES (?, ?, ?)", (name, address, phone))
    conn.commit()
    conn.close()

    return "success"

@app.route('/add_to_cart', methods=['POST'])
def add_to_cart():
    data = request.get_json()
    product_name = data['product_name']
    quantity = int(data['quantity'])

    # 최대 1개까지만 허용
    if quantity > 1:
        return jsonify(success=False), 400

    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # 존재하는 상품이면 수량 증가
    cursor.execute("SELECT quantity FROM cart_items WHERE product_name = ?", (product_name,))
    row = cursor.fetchone()
    if row:
        cursor.execute("UPDATE cart_items SET quantity = quantity + ? WHERE product_name = ?", (quantity, product_name))
    else:
        cursor.execute("INSERT INTO cart_items (product_name, quantity) VALUES (?, ?)", (product_name, quantity))

    conn.commit()
    conn.close()
    return jsonify(success=True)

@app.route('/cart_items')
def cart_items():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    cursor.execute("""
        SELECT ci.product_name, ci.quantity, p.price 
        FROM cart_items ci
        JOIN products p ON ci.product_name = p.product_name
    """)
    items = cursor.fetchall()
    conn.close()
    
    cart_data = [{'product_name': row[0], 'quantity': row[1], 'price': row[2]} for row in items]
    return jsonify(cart_data)


# DB 경로 설정
DB_PATH = os.path.join(os.path.dirname(__file__), 'database', 'database.db')

# --- ROS 2 데이터 저장을 위한 전역 변수 (Flask 앱이 접근) ---
global_ros_image_data = None # Base64 인코딩된 이미지 데이터
global_ros_result_text = "YOLO 결과를 기다리는 중..." # 결과 텍스트 (초기 메시지)

# --- ROS 2 구독 노드 클래스 (Flask 앱 내부에 통합) ---
class RosWebBridgeNode(Node):
    def __init__(self):
        super().__init__('ros_web_bridge_node')
        self.bridge = CvBridge()

        # 이미지 토픽 구독
        self.image_subscription = self.create_subscription(
            Image,
            'yolo_slot_image', # SlotCheckerNode에서 발행하는 이미지 토픽명
            self.image_callback,
            10
        )
        self.get_logger().info('/yolo_slot_image 토픽을 구독')

        # 결과 토픽 구독
        self.result_subscription = self.create_subscription(
            String,
            'yolo_slot_result', # SlotCheckerNode에서 발행하는 결과 토픽명
            self.result_callback,
            10
        )
        self.get_logger().info('/yolo_slot_result 토픽을 구독')

    def image_callback(self, msg):
        global global_ros_image_data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 이미지를 JPEG 형식으로 인코딩하고 Base64로 변환 (웹 전송용)
            ret, buffer = cv2.imencode('.jpg', cv_image)
            if ret:
                global_ros_image_data = base64.b64encode(buffer).decode('utf-8')
            # else:
            #     self.get_logger().warning('이미지를 JPEG로 인코딩하는 데 실패했습니다.')
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {e}')

    def result_callback(self, msg):
        global global_ros_result_text
        global_ros_result_text = msg.data
        # self.get_logger().info(f'결과 수신: {global_ros_result_text[:50]}...')

# --- ROS 2 노드를 별도 스레드에서 실행하는 함수 ---
def run_ros_node_thread():
    rclpy.init(args=None)
    node = RosWebBridgeNode()
    rclpy.spin(node) # ROS 2 노드가 계속 실행되면서 토픽을 구독
    node.destroy_node()
    rclpy.shutdown()

# --- ROS 2 데이터를 웹으로 제공하는 새로운 API 엔드포인트 ---
@app.route('/get_ros_data')
def get_ros_data():
    global global_ros_image_data, global_ros_result_text
    # JSON 형식으로 이미지(Base64)와 결과 텍스트 반환
    return jsonify({
        'image': global_ros_image_data,
        'result': global_ros_result_text
    })


DB_PATH = os.path.join(os.path.dirname(__file__), 'database.db')

@app.route('/get_yolo_status')
def get_yolo_status():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('SELECT slot_id, object_name, quantity, status FROM yolo_slot_status')
    rows = cursor.fetchall()
    conn.close()

    # JSON 변환
    data = []
    for row in rows:
        data.append({
            'slot_id': row[0],
            'object_name': row[1],
            'quantity': row[2],
            'status': row[3]
        })
    return jsonify(data)

@app.route('/admin')
def admin_dashboard():
    return render_template('admin.html')

def run_yolo_logger_node():
    rclpy.init()
    node = YoloResultLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Flask 앱 시작 전에 ROS 2 노드를 별도의 스레드에서 시작
    ros_thread = threading.Thread(target=run_ros_node_thread)
    ros_thread.daemon = True # 메인 스레드(Flask) 종료 시 함께 종료되도록 설정
    ros_thread.start()
    yolo_logger_thread = threading.Thread(target=run_yolo_logger_node, daemon=True)
    yolo_logger_thread.start()


    # Flask 앱 실행
    app.run(host='0.0.0.0',debug=True, port=5001)



