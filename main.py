from flask import Flask, request, jsonify, render_template, send_from_directory
from flask_cors import CORS
import json
import threading
import time
from datetime import datetime

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

# 최근 수신 데이터 저장 (최대 50개)
latest_data = []
data_lock = threading.Lock()

# 데이터 구조 초기화
def init_data():
    return {
        'timestamp': datetime.now().isoformat(),
        'flight_phase': 'IDLE',
        'altitude': 0.0,
        'imu': {
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 0.0,
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0
        },
        'motors': {
            'm1': 0.0,
            'm2': 0.0,
            'm3': 0.0,
            'm4': 0.0
        },
        'recv_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    }

current_data = init_data()

@app.route('/api/data', methods=['POST'])
def receive_data():
    try:
        data = request.get_json(force=True)
        recv_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        # 데이터 구조 업데이트
        with data_lock:
            current_data.update({
                'timestamp': datetime.fromtimestamp(data.get('timestamp', time.time()) / 1000).isoformat(),
                'flight_phase': data.get('flight_phase', 'UNKNOWN'),
                'altitude': float(data.get('altitude', 0.0)),
                'imu': {
                    'accel_x': float(data.get('imu', {}).get('accel_x', 0.0)),
                    'accel_y': float(data.get('imu', {}).get('accel_y', 0.0)),
                    'accel_z': float(data.get('imu', {}).get('accel_z', 0.0)),
                    'gyro_x': float(data.get('imu', {}).get('gyro_x', 0.0)),
                    'gyro_y': float(data.get('imu', {}).get('gyro_y', 0.0)),
                    'gyro_z': float(data.get('imu', {}).get('gyro_z', 0.0))
                },
                'motors': {
                    'm1': float(data.get('motors', {}).get('m1', 0.0)),
                    'm2': float(data.get('motors', {}).get('m2', 0.0)),
                    'm3': float(data.get('motors', {}).get('m3', 0.0)),
                    'm4': float(data.get('motors', {}).get('m4', 0.0))
                },
                'recv_time': recv_time
            })
            
            # 히스토리 저장 (최대 50개)
            latest_data.insert(0, current_data.copy())
            if len(latest_data) > 50:
                latest_data.pop()
                
        print(f"[드론 데이터 수신] {recv_time}")
        print(f"비행 상태: {current_data['flight_phase']}, 고도: {current_data['altitude']:.2f}m")
        return jsonify({'status': 'ok'}), 200
    except Exception as e:
        print(f"[서버 에러] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/api/latest')
def get_latest_data():
    with data_lock:
        return jsonify({
            'status': 'success',
            'data': current_data,
            'history': latest_data[:10]  # 최근 10개 데이터만 반환
        })

@app.route('/api/status')
def get_status():
    with data_lock:
        return jsonify({
            'status': 'success',
            'flight_phase': current_data['flight_phase'],
            'altitude': current_data['altitude'],
            'last_update': current_data['recv_time']
        })

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5003, debug=True)
