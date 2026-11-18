"""Keep the wheel/IMU EKF, the Gazebo bridge and the Nav2 speed limits consistent."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
NAV2 = ROOT.parent / "scout_mini_navigation" / "config" / "nav2_params.yaml"


def diff_drive():
    model = ET.parse(ROOT / "urdf/scout_mini_rdk.urdf.xacro")
    return next(p for p in model.iter("plugin") if p.get("name", "").endswith("DiffDrive"))


def test_ekf_is_the_only_odom_tf_publisher():
    plugin = diff_drive()
    bridge = yaml.safe_load((ROOT / "config/bridge.yaml").read_text())
    bridged = {entry["gz_topic_name"] for entry in bridge}
    # Gazebo's wheel TF must stay inside Gazebo, otherwise two nodes publish
    # odom -> base_footprint and the tree flickers between them.
    assert plugin.findtext("tf_topic") not in bridged
    assert "/tf" not in {entry["ros_topic_name"] for entry in bridge}

    ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text())
    ekf = ekf["ekf_filter_node"]["ros__parameters"]
    assert ekf["publish_tf"] is True
    assert ekf["odom_frame"] == plugin.findtext("frame_id") == "odom"
    assert ekf["base_link_frame"] == plugin.findtext("child_frame_id") == "base_footprint"
    assert ekf["odom0"] == plugin.findtext("odom_topic") in bridged
    assert ekf["imu0"] in {entry["ros_topic_name"] for entry in bridge}
    # Wheels give vx/vy only; yaw rate comes from the gyro, which does not slip.
    assert ekf["odom0_config"][11] is False
    assert ekf["imu0_config"][11] is True


def test_ekf_fuses_lio_pose_with_lag_compensation():
    ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text())
    ekf = ekf["ekf_filter_node"]["ros__parameters"]
    node = (ROOT / "scripts/lio_to_odom.py").read_text()
    assert f'"output_topic": "{ekf["odom1"]}"' in node
    # x, y and yaw from LIO; everything else comes from the wheels and the gyro.
    assert [i for i, used in enumerate(ekf["odom1_config"]) if used] == [0, 1, 5]
    # Rewinding on every out-of-order IMU/wheel message made each EKF cycle take
    # 0.1-0.2 s; lio_to_odom compensates the LIO delay before publishing instead.
    assert ekf["smooth_lagged_data"] is False
    assert "propagate(" in node


def test_nav2_limits_fit_the_simulated_base():
    plugin = diff_drive()
    params = yaml.safe_load(NAV2.read_text())
    dwb = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]
    behavior = params["behavior_server"]["ros__parameters"]
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]

    assert smoother["max_velocity"][0] == dwb["max_vel_x"] == dwb["max_speed_xy"]
    assert smoother["max_velocity"][2] == dwb["max_vel_theta"] == behavior["max_rotational_vel"]
    assert dwb["max_vel_x"] <= float(plugin.findtext("max_linear_velocity"))
    assert dwb["max_vel_theta"] <= float(plugin.findtext("max_angular_velocity"))
    # DWB drops trajectories that leave the rolling local costmap.
    assert dwb["max_vel_x"] * dwb["sim_time"] < min(local["width"], local["height"]) / 2
