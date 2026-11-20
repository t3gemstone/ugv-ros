import os
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.join(os.path.dirname(__file__), '..')
# The robot's topics, as the hardware nodes publish/subscribe them on the real robot,
# and the camera that line_sensor_sim reads instead of the line sensor array.
ROBOT_TOPICS = {'/clock', '/cmd_vel', '/wheel/odom', '/joint_states', '/imu/data_raw', '/scan',
                '/line_sensor/image'}


def _bridge():
    with open(os.path.join(PKG, 'config', 'bridge.yaml')) as f:
        return yaml.safe_load(f)


def test_world_loads_the_systems_the_robot_needs():
    world = ET.parse(os.path.join(PKG, 'worlds', 'office.sdf')).getroot().find('world')
    systems = {p.get('name').split('::')[-1] for p in world.findall('plugin')}
    # Sensors renders the lidar, Imu feeds the IMU sensor.
    assert {'Physics', 'SceneBroadcaster', 'UserCommands', 'Sensors', 'Imu'} <= systems
    assert world.find("model[@name='ground_plane']") is not None
    assert world.find("model[@name='line_track']") is not None


def test_bridge_maps_every_robot_topic_to_itself():
    entries = _bridge()
    assert {e['ros_topic_name'] for e in entries} == ROBOT_TOPICS
    for e in entries:
        assert e['ros_topic_name'] == e['gz_topic_name']
        assert e['direction'] == ('ROS_TO_GZ' if e['ros_topic_name'] == '/cmd_vel' else 'GZ_TO_ROS')


def test_sim_model_uses_robot_geometry_and_topics():
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    robot = ET.fromstring(
        xacro.process_file(os.path.join(PKG, 'urdf', 'ugv_sim.urdf.xacro')).toxml())
    description = packages.get_package_share_directory('ugv_description')
    with open(os.path.join(description, 'config', 'robot.yaml')) as f:
        geometry = yaml.safe_load(f)

    drive = robot.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert float(drive.findtext('wheel_radius')) == pytest.approx(geometry['wheel_radius'])
    assert float(drive.findtext('wheel_separation')) == pytest.approx(geometry['wheel_separation'])
    assert drive.findtext('topic') == '/cmd_vel'
    assert drive.findtext('odom_topic') == '/wheel/odom'

    sensors = {s.get('name'): s for s in robot.iter('sensor')}
    assert sensors['imu'].findtext('topic') == '/imu/data_raw'
    assert sensors['imu'].findtext('gz_frame_id') == 'imu_link'
    assert sensors['lidar'].findtext('topic') == '/scan'
    assert sensors['lidar'].findtext('gz_frame_id') == 'lidar_link'
    assert sensors['line_camera'].findtext('topic') == '/line_sensor/image'
    assert sensors['line_camera'].find('camera/image/format').text == 'L8'
