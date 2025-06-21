from flask import Flask, request, jsonify, render_template
import json
import threading
import time

app = Flask(__name__)

# 최근 수신 데이터 저장 (최대 20개)
latest_data = []
data_lock = threading.Lock()

@app.route('/api/data', methods=['POST'])
def receive_data():
    try:
        data = request.get_json(force=True)
        timestamp = data.get('timestamp')
        random_number = data.get('random_number')
        device_id = data.get('device_id')
        core_id = data.get('core_id')
        recv_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        row = {
            'recv_time': recv_time,
            'timestamp': timestamp,
            'random_number': random_number,
            'device_id': device_id,
            'core_id': core_id
        }
        with data_lock:
            latest_data.insert(0, row)
            if len(latest_data) > 20:
                latest_data.pop()
        print(f"[ESP32 데이터 수신] timestamp={timestamp}, random_number={random_number}, device_id={device_id}, core_id={core_id}")
        return jsonify({'status': 'ok'}), 200
    except Exception as e:
        print(f"[서버 에러] {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 400

@app.route('/api/latest')
def api_latest():
    with data_lock:
        return jsonify(latest_data)

@app.route('/')
def index():
    return render_template('index.html')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5003)
