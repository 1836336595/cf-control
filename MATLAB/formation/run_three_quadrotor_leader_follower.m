function sim = run_three_quadrotor_leader_follower()
%RUN_THREE_QUADROTOR_LEADER_FOLLOWER 三机领导-跟随 8 字轨迹仿真。
%
% 虚拟领导者是三架飞机的几何中心，不是一架物理飞机。三架飞机均使用
% 中心 pinning 和邻居相对误差的分布式控制律，不设置物理领航机。

clc;
close all;

cfg = formation_three_quadrotor_parameters();
cfg.formation.type = 'leader_follower';
cfg.simulation.duration = 22;
cfg.simulation.dt = 0.004;
cfg.attitudeController.omegaCMethod = 'log_difference';
cfg.visualization.plot = true;
cfg.visualization.animate = true;

% 相对虚拟中心的固定偏移，列和为零，所以 p_c 是几何中心。
cfg.formation.offsets = [-0.35,  0.175,  0.175; ...
                          0.00, -0.303,  0.303; ...
                          0.00,  0.000,  0.000];
cfg.formation.adjacency = [0, 1, 1; ...
                           1, 0, 1; ...
                           1, 1, 0];
cfg.formation.kf = 2.8;
cfg.formation.kvf = 2.1;
cfg.formation.kbl = 2.0;
cfg.formation.kvl = 1.6;
% 三架飞机都能获得虚拟中心参考，因此都是分布式中心 pinning 节点。
cfg.formation.bl = [1.0; 1.0; 1.0];

cfg.referenceFcn = @formation_figure_eight_reference;
[initialCenterPosition, initialCenterVelocity] = ...
    formation_figure_eight_reference(0);
positionPerturbation = [ 0.02, -0.01, -0.01; ...
                        -0.01,  0.02, -0.01; ...
                         0.01, -0.02,  0.01];
cfg.initial.position = repmat(initialCenterPosition(:), 1, 3) + ...
    cfg.formation.offsets + positionPerturbation;
cfg.initial.velocity = repmat(initialCenterVelocity(:), 1, 3);
cfg.initial.R = repmat(eye(3), 1, 1, 3);
cfg.initial.bodyRate = zeros(3, 3);

sim = formation_three_quadrotor_simulation(cfg);
end
