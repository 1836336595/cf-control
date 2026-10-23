#!/usr/bin/python3
"""基于 Nokov 位姿与固件 EKF 运动学的 Crazyflie 主机端几何 CTBR 外环控制器。

控制分工
========

本文件只运行在主机。Nokov 位姿由 ``crazyswarm_server`` 转换为
``/cf<ID>/mocap_state``；本节点计算总推力 ``T``（N）和机体系角速度
``[p, q, r]``（rad/s），连续发布到 ``/cf<ID>/cmd_ctbr``。服务器再将它转换为
legacy RPYT CRTP 包，机载固件负责角速度内环、电机混控和姿态估计。
Nokov 始终只提供位置和 ``R_WB``。速度和由速度导出的加速度只来自固件 EKF；EKF
位置与 Nokov 的差值仅用作健康检查，绝不把 Nokov 差分速度/加速度切入 CTBR 反馈。

坐标与单位
==========

world 坐标系采用 ROS 约定，z 轴向上。刚体姿态矩阵 ``R`` 将机体系向量转换到 world
系；机体 z 轴 ``R[:, 2]`` 指向螺旋桨产生正推力的方向。位置单位为 m，速度为 m/s，
角速度为 rad/s，推力为整机总推力 N。

运行模式与安全边界
==================

本节点只运行实际 CTBR 控制流程。启动前必须设置 ``~target_confirmed:=true``；状态失效、
状态超时和进程退出时均发送零 CTBR。起飞、轨迹和降落均在同一个低层流式控制器内完成，
不能在发送 CTBR 后再调用高层 ``takeoff()`` 或 ``land()``。
"""

import time
import rospy

from ctbr_parameters import (CtbrParameterResolver, vehicle_parameter_namespace,
                             GLOBAL_CTBR_PARAMETER_ROOT, TRAJECTORY_PARAMETER_ROOT,
                             PAYLOAD_PARAMETER_ROOT)
from ctbr_control_math import (
    as_vector, clamp_vector, vee, project_so3, rope_preserving_vehicle_references,
    so3_log, _limit_force_with_derivative, analytic_omega_c,
    quaternion_to_rotation, rotation_to_rpy, phase_requires_full_controller_reset,
)
from ctbr_state_feedback import SecondOrderVelocityFilter, blend_kinematic_feedback
from ctbr_geometric_controller import ControllerConfig, GeometricCtbrController
from ctbr_csv_logger import FlightCsvLogger
from ctbr_vehicle_node import CtbrControllerNode
from ctbr_fleet_node import MultiCtbrControllerNode
# Keep existing test/replay imports public while their implementations live in
# the corresponding pure load and reference modules.
from collections import deque

from vehicle_config import (
    select_vehicle_entries,
    select_vehicle_entry,
    validate_vehicle_entries,
    vehicle_uri,
)

from slung_prelift_trajectory import SlungPreliftTrajectory, handoff_references

from slung_transport_guard import SlungTransportGuard, SlungSupportTrim

from slung_transport_integral import SlungDifferentialHoldIntegral

from native_position_pid import (position_integral_step, position_pid_feedback,
                                 validate_integral_velocity_weight)

from slung_landing import SlungLandingProfile, PayloadTouchdownDetector

from slung_link_calibration import SlungLinkCalibrator

from slung_reference import attachment_rotation_reference, cable_direction_reference

from slung_circle_reference import (
    CircleTrajectoryConfig, PayloadCircleReference, PayloadFigureEightReference,
    AttitudeBrakeReference, rotate_vehicle_references,
)

from slung_attitude_trim import SlungAttitudeReferenceTrim

from ctbr_trajectory import (
    CircularFlightTrajectory,
    CircularTrajectoryConfig,
)

from slung_load_controller import (
    SlungLoadConfig,
    SlungLoadController,
    PayloadStateObserver,
    SecondOrderVelocityFilter as PayloadVelocityFilter,
    attachment_points_from_yaml,
    cable_distances,
    link_lengths_vector,
    mocap_top_surface_to_center,
    ordered_vehicle_ids,
    payload_ground_clearance,
    smoothstep5_profile,
    takeup_distance_ready,
    takeup_target_distance,
    takeup_vehicle_targets,
    translate_vehicle_references,
)


if __name__ == "__main__":
    rospy.init_node("ctbr_controller")
    MultiCtbrControllerNode()
    rospy.spin()
