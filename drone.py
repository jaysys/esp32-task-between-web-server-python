from flask import Flask, request, jsonify, render_template, send_from_directory
from flask_cors import CORS
import json
import threading
import time
from collections import deque
import math
from datetime import datetime
from collections import deque

# For calculating orientation from accelerometer and gyroscope data
from scipy.spatial.transform import Rotation as R

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# 최근 수신 데이터 저장 (최대 50개)
latest_data = deque(maxlen=50)
data_lock = threading.Lock()

# 비행 명령 큐
command_queue = deque()
command_lock = threading.Lock()

# 드론 상태 초기화
drone_state = {
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # x, y, z 좌표 (m)
    'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # x, y, z 속도 (m/s)
    'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},  # radian
    'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}  # rad/s
}

# 데이터 구조 초기화
def init_data():
    return {
        'timestamp': datetime.now().isoformat(),
        'flight_phase': 'IDLE',
        'altitude': 0.0,
        'position': drone_state['position'].copy(),
        'orientation': drone_state['orientation'].copy(),
        'imu': {
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 9.8,  # 중력가속도 (m/s²)
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0,
            'roll_rad': 0.0,  # 추가: roll 각도 (rad)
            'pitch_rad': 0.0,  # 추가: pitch 각도 (rad)
            'yaw_rad': 0.0,   # 추가: yaw 각도 (rad)
        },
        'motors': {
            'm1': 0.0, 'm2': 0.0, 'm3': 0.0, 'm4': 0.0
        },
        'recv_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }

current_data = init_data()

def calculate_orientation(accel, gyro, dt=0.1):
    """가속도계와 자이로스코프 데이터로부터 방향 계산"""
    # 자이로스코프 적분
    roll = drone_state['orientation']['roll'] + gyro['x'] * dt
    pitch = drone_state['orientation']['pitch'] + gyro['y'] * dt
    yaw = drone_state['orientation']['yaw'] + gyro['z'] * dt
    
    # 가속도계로 보정 (상보필터 적용)
    alpha = 0.98
    if abs(accel['x']) + abs(accel['y']) + abs(accel['z']) > 0:
        # 가속도계로부터의 roll, pitch 계산
        accel_pitch = math.atan2(accel['y'], math.sqrt(accel['x']**2 + accel['z']**2))
        accel_roll = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2))
        
        # 상보필터 적용
        roll = roll * alpha + accel_roll * (1 - alpha)
        pitch = pitch * alpha + accel_pitch * (1 - alpha)
    
    return {
        'roll': roll,
        'pitch': pitch,
        'yaw': yaw
    }

def process_flight_commands():
    """비행 명령 처리"""
    while True:
        with command_lock:
            if command_queue:
                command = command_queue.popleft()
                # 여기에 명령 처리 로직 추가
                print(f"Processing command: {command}")
        time.sleep(0.1)  # CPU 사용량을 줄이기 위한 대기

# 비행 명령 처리 스레드 시작
command_thread = threading.Thread(target=process_flight_commands, daemon=True)
command_thread.start()

# Command handling is now moved below with improved implementation

@app.route('/api/data', methods=['POST'])
def receive_data():
    try:
        data = request.get_json(force=True)
        recv_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        print(f"[DEBUG] Received data from drone: {data}")
        
        # Get IMU data from the request
        imu_data = data.get('imu', {})
        
        accel = {
            'x': float(imu_data.get('accel_x', 0.0)),
            'y': float(imu_data.get('accel_y', 0.0)),
            'z': float(imu_data.get('accel_z', 9.8))
        }
        gyro = {
            'x': float(imu_data.get('gyro_x', 0.0)),
            'y': float(imu_data.get('gyro_y', 0.0)),
            'z': float(imu_data.get('gyro_z', 0.0))
        }
        
        # 방향 계산
        orientation = calculate_orientation(accel, gyro)
        
        # 드론 상태 업데이트
        with data_lock:
            drone_state['orientation'] = orientation
            drone_state['angular_velocity'] = gyro
            
            # 위치 업데이트 (단순화된 모델)
            dt = 0.1  # 초 단위
            drone_state['position']['x'] += drone_state['velocity']['x'] * dt
            drone_state['position']['y'] = float(data.get('altitude', 0.0))  # 고도 업데이트
            drone_state['position']['z'] += drone_state['velocity']['z'] * dt
        
        # 데이터 구조 업데이트
        with data_lock:
            current_data.update({
                'timestamp': datetime.fromtimestamp(data.get('timestamp', time.time()) / 1000).isoformat(),
                'flight_phase': data.get('flight_phase', 'UNKNOWN'),
                'altitude': float(data.get('altitude', 0.0)),
                'position': drone_state['position'].copy(),
                'orientation': orientation,
                'imu': {
                    'accel_x': accel['x'],
                    'accel_y': accel['y'],
                    'accel_z': accel['z'],
                    'gyro_x': gyro['x'],
                    'gyro_y': gyro['y'],
                    'gyro_z': gyro['z'],
                    'roll_rad': orientation['roll'],
                    'pitch_rad': orientation['pitch'],
                    'yaw_rad': orientation['yaw']
                },
                'motors': data.get('motors', {
                    'm1': 0.0,
                    'm2': 0.0,
                    'm3': 0.0,
                    'm4': 0.0
                }),
                'recv_time': recv_time
            })
            
            # 히스토리 저장 (최대 50개)
            # 먼저 새로운 데이터를 추가하기 전에 크기 확인
            if len(latest_data) >= 50:
                latest_data.pop()  # 오래된 데이터 제거
            latest_data.appendleft(current_data.copy())  # 새로운 데이터 추가
                
        print(f"[ESP32 데이터 수신] {recv_time}")
        return jsonify({'status': 'ok'}), 200
        
    except Exception as e:
        print(f"[서버 에러] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/api/latest')
def get_latest_data():
    with data_lock:
        # Convert deque to list before slicing
        history_list = list(latest_data)[:10]  # 최근 10개 데이터만 반환
        response_data = {
            'status': 'success',
            'data': current_data,
            'history': history_list
        }
        print(f"[DEBUG] Sending latest data: {response_data}")
        return jsonify(response_data)

@app.route('/api/status')
def get_status():
    with data_lock:
        return jsonify({
            'status': 'success',
            'flight_phase': current_data['flight_phase'],
            'altitude': current_data['altitude'],
            'motors': current_data.get('motors', {
                'm1': 0.0, 'm2': 0.0, 'm3': 0.0, 'm4': 0.0
            }),
            'timestamp': current_data.get('timestamp'),  # 드론에서 보낸 원본 타임스탬프
            'last_update': current_data['recv_time']  # 서버에서 수신한 시간
        })

# 정적 파일 서빙
@app.route('/static/<path:path>')
def send_static(path):
    return send_from_directory('static', path)

@app.route('/')
def index():
    return render_template('drone-index.html')

# Initialize data structures
current_data = init_data()
latest_data = deque(maxlen=50)  # 최신 데이터 50개 저장
data_lock = threading.Lock()

# 명령 큐
command_queue = []
command_lock = threading.Lock()

# CORS 설정
CORS(app, resources={
    r"/api/*": {
        "origins": "*",
        "methods": ["GET", "POST", "OPTIONS"],
        "allow_headers": ["Content-Type"]
    }
})

@app.route('/api/command', methods=['POST'])
def handle_command():
    try:
        data = request.get_json()
        command = data.get('command')
        
        if not command:
            return jsonify({'status': 'error', 'message': 'No command provided'}), 400
            
        with command_lock:
            command_queue.append(command)
            
        print(f"[COMMAND] Received command: {command}")
        return jsonify({
            'status': 'success', 
            'command': command,
            'message': f'Command {command} queued'
        })
        
    except Exception as e:
        error_msg = f"[Command Error] {e}"
        print(error_msg)
        return jsonify({
            'status': 'error', 
            'message': str(e),
            'details': error_msg
        }), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5003, debug=True)
