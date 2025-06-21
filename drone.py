from flask import Flask, request, jsonify, render_template, send_from_directory
from flask_cors import CORS
import json
import threading
import time
from datetime import datetime

app = Flask(__name__)

# 최근 수신 데이터 저장 (최대 50개)
latest_data = []
data_lock = threading.Lock()

# 데이터 구조 초기화
def init_data():
    return {
        'timestamp': datetime.now().isoformat(),
        'sensor': {
            'imu': {'accel': {'x': 0, 'y': 0, 'z': 0}, 'gyro': {'x': 0, 'y': 0, 'z': 0}},
            'baro': {'altitude': 0},
            'gps': {'lat': 0, 'lon': 0, 'alt': 0, 'speed': 0},
            'magnetometer': {'x': 0, 'y': 0, 'z': 0}
        },
        'stabilizer': {
            'attitude': {'pitch': 0, 'roll': 0, 'yaw': 0},
            'pid': {'pitch': 0, 'roll': 0, 'yaw': 0},
            'motors': {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0}
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
                'timestamp': datetime.now().isoformat(),
                'sensor': data.get('sensor', {}),
                'stabilizer': data.get('stabilizer', {}),
                'recv_time': recv_time
            })
            
            # 히스토리 저장 (최대 50개)
            latest_data.insert(0, current_data.copy())
            if len(latest_data) > 50:
                latest_data.pop()
                
        print(f"[ESP32 데이터 수신] {recv_time}")
        return jsonify({'status': 'ok'}), 200
        
    except Exception as e:
        print(f"[서버 에러] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/api/current')
def api_current():
    with data_lock:
        return jsonify({
            'current': current_data,
            'history': latest_data[:10]  # Return last 10 history items for reference
        })

# 정적 파일 서빙
@app.route('/static/<path:path>')
def send_static(path):
    return send_from_directory('static', path)

@app.route('/')
def index():
    return render_template('drone-index.html')

if __name__ == "__main__":
    CORS(app)  # CORS 허용
    app.run(host='0.0.0.0', port=5003, debug=True)
