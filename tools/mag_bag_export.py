"""Chay tren Pi: xuat /mavros/imu/mag, data_raw, data, gps raw tu bag ra CSV cho mag_bag_to_npz.py."""
import sys, glob
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

files = sorted(glob.glob(sys.argv[1] + '/*.mcap'))
T = {'/mavros/imu/mag', '/mavros/imu/data_raw', '/mavros/imu/data', '/mavros/gpsstatus/gps1/raw'}
out = open(sys.argv[2], 'w')
for f in files:
    r = rosbag2_py.SequentialReader()
    try:
        r.open(rosbag2_py.StorageOptions(uri=f, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    except Exception as e:
        print('bo', f, e); continue
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(T & set(types))))
    n = 0
    try:
        while r.has_next():
            topic, data, tbag = r.read_next()
            m = deserialize_message(data, get_message(types[topic]))
            if topic == '/mavros/gpsstatus/gps1/raw':
                out.write(f'gps,{tbag*1e-9:.4f},{m.fix_type},{m.satellites_visible},{m.h_acc},{m.lat},{m.lon},{m.alt}\n')
                continue
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if topic.endswith('mag'):
                v = m.magnetic_field; out.write(f'mag,{s:.6f},{v.x:.9e},{v.y:.9e},{v.z:.9e}\n')
            elif topic.endswith('data_raw'):
                g, a = m.angular_velocity, m.linear_acceleration
                out.write(f'imu,{s:.6f},{g.x:.6f},{g.y:.6f},{g.z:.6f},{a.x:.5f},{a.y:.5f},{a.z:.5f}\n')
            else:
                q = m.orientation; out.write(f'att,{s:.6f},{q.x:.6f},{q.y:.6f},{q.z:.6f},{q.w:.6f}\n')
            n += 1
    except Exception as e:
        print('loi doc giua', f, e)
    print(f, n)
