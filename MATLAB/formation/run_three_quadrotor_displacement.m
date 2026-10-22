function sim = run_three_quadrotor_displacement()
%RUN_THREE_QUADROTOR_DISPLACEMENT 三机位移型编队 8 字轨迹仿真。
%
% 直接在 MATLAB R2021b 中运行本文件即可。程序使用本目录中的共用
% formation_three_quadrotor_simulation、V2 CTBR 控制器和简化刚体模型。
% 1 号机使用几何控制单机跟踪 8 字参考；2、3 号机只使用位移型
% 分布式相对位置/速度一致性控制，并通过共同加速度前馈跟随 8 字。

clc;
close all;

cfg = formation_three_quadrotor_parameters();
cfg.formation.type = 'displacement';
cfg.simulation.duration = 22;
cfg.simulation.dt = 0.004;
cfg.attitudeController.omegaCMethod = 'log_difference';
cfg.visualization.plot = true;
cfg.visualization.animate = true;

% 三机固定偏移的列和为零，1 号机是几何领航机，编队中心仍是 8 字轨迹。
cfg.formation.offsets = [-0.35,  0.175,  0.175; ...
                          0.00, -0.303,  0.303; ...
                          0.00,  0.000,  0.000];
cfg.formation.displacementLeaderIndex = 1;

% 无向全连接图。a(i,j)=1 表示 i 使用 j 的相对信息。
cfg.formation.adjacency = [0, 1, 1; ...
                           1, 0, 1; ...
                           1, 1, 0];
cfg.formation.kp = 2.6;
cfg.formation.kv = 2.0;

% 用速度方向作为目标航向；在 8 字速度接近零的点由轨迹函数保持航向连续。
cfg.referenceFcn = @formation_figure_eight_reference;

% 初始位置以参考轨迹和固定偏移为基准，并加入零质心的小扰动，检验
% 几何领航机和相对位移一致性控制的收敛过程。
[initialCenterPosition, initialCenterVelocity] = ...
    formation_figure_eight_reference(0);
positionPerturbation = [ 0.02, -0.01, -0.01; ...
                        -0.01,  0.02, -0.01; ...
                         0.01, -0.02,  0.01];
cfg.initial.position = repmat(initialCenterPosition(:), 1, 3) + ...
    cfg.formation.offsets + positionPerturbation;
% 三架飞机的初速度设为 8 字参考在 t=0 的速度。
cfg.initial.velocity = repmat(initialCenterVelocity(:), 1, 3);
cfg.initial.R = repmat(eye(3), 1, 1, 3);
cfg.initial.bodyRate = zeros(3, 3);

sim = formation_three_quadrotor_simulation(cfg);
end
