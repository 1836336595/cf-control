std_msgs/Header header
geometry_msgs/Vector3 body_rates
float32 collective_thrust
# 仅作用于主机桥接层的 raw PWM 映射；0 表示兼容旧发布者并使用 1.0。
# 控制器在起飞前由冻结的电压中位数计算该值，飞行中不得逐样本改变。
float32 thrust_raw_scale
