#!/usr/bin/env python3
"""
FinOrchard Pro - Webhook server
Hanterar Arduino MKR WAN 1310 och KIWI-sensorer
"""

from flask import Flask, request, jsonify
from datetime import datetime
import sqlite3
import json
import math
import os

app = Flask(__name__)
DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'finorchard.db')

def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS arduino_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TEXT NOT NULL,
        device_id TEXT NOT NULL,
        dev_addr TEXT,
        f_cnt INTEGER,
        rssi REAL,
        snr REAL,
        wm1_raw INTEGER,
        wm2_raw INTEGER,
        temp1_c REAL,
        temp2_c REAL,
        bl1_raw INTEGER,
        bl2_raw INTEGER,
        tb_tips INTEGER,
        tb_mm REAL,
        air_temp_c REAL,
        air_rh_pct INTEGER,
        dew_point_c REAL,
        gateway_id TEXT,
        raw_payload TEXT
    )''')
    c.execute('''CREATE TABLE IF NOT EXISTS kiwi_data (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        timestamp TEXT NOT NULL,
        device_id TEXT NOT NULL,
        dev_addr TEXT,
        f_cnt INTEGER,
        rssi REAL,
        snr REAL,
        temp1_c REAL,
        temp2_c REAL,
        wm1_raw REAL,
        wm2_raw REAL,
        air_temp_c REAL,
        air_rh_pct REAL,
        light_lux REAL,
        mcu_temp_c REAL,
        battery_pct REAL,
        battery_days REAL,
        gateway_id TEXT,
        raw_payload TEXT
    )''')
    conn.commit()
    conn.close()

def dew_point(temp_c, rh_pct):
    if temp_c is None or rh_pct is None or rh_pct <= 0:
        return None
    try:
        a, b = 17.271, 237.7
        gamma = (a * temp_c / (b + temp_c)) + math.log(rh_pct / 100.0)
        return round((b * gamma) / (a - gamma), 2)
    except:
        return None

def is_arduino(device_id):
    return device_id.lower().startswith('arduino')

def save_arduino(device_id, dev_addr, f_cnt, rssi, snr, gateway_id, msg, raw):
    dp = msg.get('decoded_payload', {})
    air_temp = dp.get('air_temp_c')
    air_rh = dp.get('air_rh_pct')
    temp1 = dp.get('temp1_c')
    temp2 = dp.get('temp2_c')
    if air_temp is not None and (air_temp < -40 or air_temp > 85):
        air_temp = None
    if air_rh is not None and (air_rh <= 0 or air_rh > 100):
        air_rh = None
    if temp1 is not None and (temp1 < -100 or temp1 > 50):
        temp1 = None
    if temp2 is not None and (temp2 < -100 or temp2 > 50):
        temp2 = None
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''INSERT INTO arduino_data (
        timestamp, device_id, dev_addr, f_cnt, rssi, snr,
        wm1_raw, wm2_raw, temp1_c, temp2_c,
        bl1_raw, bl2_raw, tb_tips, tb_mm,
        air_temp_c, air_rh_pct, dew_point_c,
        gateway_id, raw_payload
    ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)''', (
        datetime.utcnow().isoformat(),
        device_id, dev_addr, f_cnt, rssi, snr,
        dp.get('wm1_raw'), dp.get('wm2_raw'),
        temp1, temp2,
        dp.get('bl1_raw'), dp.get('bl2_raw'),
        dp.get('tb_tips'), dp.get('tb_mm'),
        air_temp, air_rh, dew_point(air_temp, air_rh),
        gateway_id, json.dumps(raw)
    ))
    conn.commit()
    conn.close()

def save_kiwi(device_id, dev_addr, f_cnt, rssi, snr, gateway_id, msg, raw):
    dp = msg.get('decoded_payload', {})
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''INSERT INTO kiwi_data (
        timestamp, device_id, dev_addr, f_cnt, rssi, snr,
        temp1_c, temp2_c, wm1_raw, wm2_raw,
        air_temp_c, air_rh_pct,
        light_lux, mcu_temp_c,
        battery_pct, battery_days,
        gateway_id, raw_payload
    ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)''', (
        datetime.utcnow().isoformat(),
        device_id, dev_addr, f_cnt, rssi, snr,
        dp.get('temp1_c'), dp.get('temp2_c'),
        dp.get('wm1_raw'), dp.get('wm2_raw'),
        dp.get('air_temp_c'), dp.get('air_rh_pct'),
        dp.get('light_lux'), dp.get('mcu_temp_c'),
        dp.get('battery_pct'), dp.get('battery_days'),
        gateway_id, json.dumps(raw)
    ))
    conn.commit()
    conn.close()

@app.route('/')
def home():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('SELECT COUNT(*) FROM arduino_data')
    arduino_count = c.fetchone()[0]
    c.execute('SELECT COUNT(*) FROM kiwi_data')
    kiwi_count = c.fetchone()[0]
    c.execute('SELECT timestamp, device_id, temp1_c, air_temp_c, air_rh_pct, dew_point_c, wm1_raw, wm2_raw, bl1_raw, bl2_raw FROM arduino_data ORDER BY id DESC LIMIT 5')
    arduino_latest = c.fetchall()
    c.execute('SELECT timestamp, device_id, temp1_c, temp2_c, wm1_raw, wm2_raw, light_lux, mcu_temp_c FROM kiwi_data ORDER BY id DESC LIMIT 5')
    kiwi_latest = c.fetchall()
    conn.close()

    arduino_rows = ''
    for r in arduino_latest:
        arduino_rows += f'<tr><td>{r[0][:19]}</td><td>{r[1]}</td><td>{r[2]}</td><td>{r[3]}</td><td>{r[4]}</td><td>{r[5]}</td><td>{r[6]}/{r[7]}</td><td>{r[8]}/{r[9]}</td></tr>'

    kiwi_rows = ''
    for r in kiwi_latest:
        kiwi_rows += f'<tr><td>{r[0][:19]}</td><td>{r[1]}</td><td>{r[2]}</td><td>{r[3]}</td><td>{r[4]}/{r[5]}</td><td>{r[6]}</td><td>{r[7]}</td></tr>'

    return f'''<!DOCTYPE html>
<html><head><title>FinOrchard Pro</title>
<style>
body{{font-family:Arial;margin:20px;background:#f0f4f0}}
h1{{color:#2d6a2d}}h2{{color:#2d6a2d;margin-top:30px}}
table{{border-collapse:collapse;width:100%;margin-bottom:20px}}
th,td{{border:1px solid #ccc;padding:6px;font-size:13px}}
th{{background:#2d6a2d;color:white}}
.stat{{display:inline-block;background:white;padding:15px 25px;margin:10px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);text-align:center}}
.stat b{{font-size:24px;display:block;color:#2d6a2d}}
</style></head><body>
<h1>FinOrchard Pro</h1>
<div class="stat"><b>{arduino_count}</b>Arduino matningar</div>
<div class="stat"><b>{kiwi_count}</b>KIWI matningar</div>
<h2>Arduino-noder (senaste 5)</h2>
<table><tr><th>Tid</th><th>Enhet</th><th>Temp1 C</th><th>Lufttemp C</th><th>RH %</th><th>Daggpunkt C</th><th>WM1/WM2</th><th>BL1/BL2</th></tr>
{arduino_rows if arduino_rows else '<tr><td colspan=8>Ingen data</td></tr>'}
</table>
<h2>KIWI-sensorer (senaste 5)</h2>
<table><tr><th>Tid</th><th>Enhet</th><th>Temp1 C</th><th>Temp2 C</th><th>WM1/WM2 kHz</th><th>Ljus lux</th><th>MCU C</th></tr>
{kiwi_rows if kiwi_rows else '<tr><td colspan=7>Ingen data</td></tr>'}
</table>
<p><small>API: <a href="/api/arduino">/api/arduino</a> | <a href="/api/kiwi">/api/kiwi</a> | <a href="/api/stats">/api/stats</a> | <a href="/health">/health</a></small></p>
</body></html>'''

@app.route('/webhook', methods=['POST'])
def webhook():
    try:
        data = request.get_json(force=True)
        if not data:
            return jsonify({'error': 'No data'}), 400
        ids = data.get('end_device_ids', {})
        device_id = ids.get('device_id', 'unknown')
        dev_addr = ids.get('dev_addr', '')
        msg = data.get('uplink_message', {})
        f_cnt = msg.get('f_cnt', 0)
        rx = msg.get('rx_metadata', [{}])[0]
        rssi = rx.get('rssi')
        snr = rx.get('snr')
        gateway_id = rx.get('gateway_ids', {}).get('gateway_id', '')
        if is_arduino(device_id):
            save_arduino(device_id, dev_addr, f_cnt, rssi, snr, gateway_id, msg, data)
            sensor_type = 'arduino'
        else:
            save_kiwi(device_id, dev_addr, f_cnt, rssi, snr, gateway_id, msg, data)
            sensor_type = 'kiwi'
        return jsonify({'success': True, 'device': device_id, 'type': sensor_type}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/arduino')
def api_arduino():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute('SELECT * FROM arduino_data ORDER BY id DESC LIMIT 50')
    rows = [dict(r) for r in c.fetchall()]
    conn.close()
    return jsonify(rows)

@app.route('/api/kiwi')
def api_kiwi():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    c = conn.cursor()
    c.execute('SELECT * FROM kiwi_data ORDER BY id DESC LIMIT 50')
    rows = [dict(r) for r in c.fetchall()]
    conn.close()
    return jsonify(rows)

@app.route('/api/stats')
def api_stats():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('SELECT COUNT(*) FROM arduino_data')
    arduino = c.fetchone()[0]
    c.execute('SELECT COUNT(*) FROM kiwi_data')
    kiwi = c.fetchone()[0]
    conn.close()
    return jsonify({'arduino_readings': arduino, 'kiwi_readings': kiwi})

@app.route('/health')
def health():
    return jsonify({'status': 'ok'})

init_db()

if __name__ == '__main__':
    app.run(debug=True, port=5003)
