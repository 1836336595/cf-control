# MATLAB 吊运控制器接入设计

## 目标

将 `MATLAB/Multi-UAV-Transportation-Simulation` 的三机吊运控制器接入当前 Crazyswarm CTBR 主机控制链路。负载刚体名称固定为 `load`，NOKOV 刚体原点位于负载上表面中心，控制器根据长方体尺寸换算几何中心。ROS world 坐标系 z 轴向上，MATLAB 惯性系 z 轴向下，所有输入、控制力和挂点在 Python 控制器内部统一转换。

## 三机映射

运输控制器固定按 `[cf3, cf4, cf5]` 建立矩阵列顺序：CF3 位于负载 +x 棱中点，CF4 位于左下挂点，CF5 位于右下挂点。挂点参数使用负载坐标系，转换到 ROS world 时通过负载姿态旋转；z-down 的 MATLAB 挂点 z 坐标取反后作为 ROS z-up 坐标。

## 状态接口

`crazyswarm_server` 从 NOKOV 刚体表中查找 `load`，发布 `/load/mocap_state`。该消息中的位姿是 NOKOV 原始刚体位姿，速度和加速度由服务端按当前 MocapState 估计器产生。Python 端将原点位置加上 `R_WB * [0,0,-height/2]` 得到几何中心位置；姿态矩阵保持不变。

## 控制结构

起飞、高度校正、入轨和降落继续使用现有安全轨迹。运输阶段使用 MATLAB 链路：负载位置/姿态外环、张力分配、绳向控制、每架飞机总合力、几何姿态和角速度指令。运输阶段不叠加当前虚拟中心 `formation_kf/kvf/kbl/kvl` 控制，也不叠加每机位置 PID。

## 配置

- `config/slung_payload.yaml`：负载质量、尺寸、惯量、挂点、绳长、负载外环、张力分配和绳向控制参数。
- `config/ctbr_vehicle.yaml`：每架 Crazyflie 的质量、推力标定、姿态/速率参数和 `payload_attachment_index`。
- `ctbr_controller.yaml` 保留任务调度、轨迹、EKF 和日志参数，并加载以上两个参数块。

## 安全行为

在 `/load/mocap_state` 无效、超时或几何中心换算非有限时，所有飞机进入等待/安全零推力路径。负载状态恢复且三架飞机状态均有效后，才允许进入运输阶段。

