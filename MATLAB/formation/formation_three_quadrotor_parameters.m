function cfg = formation_three_quadrotor_parameters(userCfg)
%FORMATION_THREE_QUADROTOR_PARAMETERS 三机编队仿真的集中参数文件。
%
% 坐标约定与 simulator/CTBR/codex/v2 完全一致：
%   * 惯性系 e3=[0;0;1] 指向重力，位置 z 轴向下；
%   * R 为机体系到惯性系的旋转矩阵；
%   * m*v_dot = m*g*e3 - f*R*e3；
%   * 推力 f 为非负标量，CTBR 外环发送推力百分比和机体角速度。

if nargin < 1 || isempty(userCfg)
    userCfg = struct();
end

nVehicles = 3;

cfg.simulation = struct( ...
    'duration', 22.0, ...
    'dt', 0.004, ...
    'maxBodyRate', [8; 8; 6]);

cfg.vehicle = struct();
cfg.vehicle.mass = 0.037 * ones(nVehicles, 1);
cfg.vehicle.gravity = 9.81;
cfg.vehicle.inertia = repmat(diag([1.8e-5, 1.8e-5, 3.0e-5]), ...
    1, 1, nVehicles);
cfg.vehicle.maxTotalThrust = 0.75 * ones(nVehicles, 1);
cfg.vehicle.thrustTimeConstant = 0.035;
cfg.vehicle.externalForce = zeros(3, nVehicles);

% 两种编队共用的通信图和固定偏移。三列分别对应三架飞机，且偏移和为零，
% 因而 centerPosition 才是三架飞机几何中心的参考位置。
cfg.formation = struct();
cfg.formation.type = 'displacement';
cfg.formation.offsets = [-0.35,  0.175,  0.175; ...
                          0.00, -0.303,  0.303; ...
                          0.00,  0.000,  0.000];
cfg.formation.adjacency = [0, 1, 1; ...
                           1, 0, 1; ...
                           1, 1, 0];

% 位移型分布式一致性控制的 kp/kv。默认 displacement 模式中，1 号机
% 使用单机几何位置控制，2/3 号机使用下面的相对位移/速度一致性控制。
cfg.formation.kp = 2.6;
cfg.formation.kv = 2.0;
cfg.formation.displacementLeaderIndex = 1;
cfg.formation.commonAcceleration = true;

% 领导-跟随型使用虚拟中心点作为领导者。三架飞机均可获得中心点信息，
% 因而 bl=[1;1;1] 时三架飞机全部使用中心 pinning 的分布式控制律。
cfg.formation.kf = 2.8;
cfg.formation.kvf = 2.1;
cfg.formation.kbl = 2.0;
cfg.formation.kvl = 1.6;
cfg.formation.bl = ones(nVehicles, 1);

% 1 号几何领航机使用几何控制单机.pdf/V2 的位置外环参数。
cfg.geometricController = struct( ...
    'kx', 2.8, ...
    'kv', 2.2, ...
    'ki', 0.12, ...
    'c1', 0.6, ...
    'integralLimit', [0.35; 0.35; 0.35]);

% CTBR 姿态外环。它与 V2 控制器保持相同的误差定义和限幅方式。
% 'omegaCMethod', analytic 或 log_difference
cfg.attitudeController = struct( ...
    'kr', [3.0; 3.0; 2.0], ...
    'useIntegral', false, ...
    'ki', [0.15; 0.15; 0.10], ...
    'integralLimit', [0.5; 0.5; 0.5], ...
    'maxBodyRateCommand', [4.5; 4.5; 3.5], ...
    'omegaCMethod', 'analytic');

cfg.omegaC = struct( ...
    'forceNormEpsilon', 1e-7, ...
    'headingProjectionEpsilon', 1e-6);

% 仿真的 Crazyflie 内部角速度环。
cfg.rateController = struct( ...
    'kp', [10; 10; 7], ...
    'ki', [3; 3; 2], ...
    'integralLimit', [1.5; 1.5; 1.0], ...
    'maxMoment', [0.003; 0.003; 0.0015]);

% 初始状态。入口脚本通常会覆盖位置，但保留一组可直接运行的默认值。
cfg.initial = struct();
cfg.initial.position = [0.10, -0.50, -0.58; ...
                        -0.42, -0.48, -0.57; ...
                         0.12,  0.02, -0.64]';
cfg.initial.velocity = zeros(3, nVehicles);
cfg.initial.R = repmat(eye(3), 1, 1, nVehicles);
cfg.initial.bodyRate = zeros(3, nVehicles);
cfg.initial.thrustNewton = cfg.vehicle.mass * cfg.vehicle.gravity;

% 参考轨迹函数遵循 V2 六输出顺序 [pc, vc, ac, Rd, jerkc, RdDot]。
cfg.referenceFcn = @formation_figure_eight_reference;

cfg.visualization = struct( ...
    'plot', true, ...
    'animate', true, ...
    'animationStride', 8, ...
    'quadrotorArmLength', 0.09, ...
    'axisPadding', 0.35, ...
    'saveVideo', false, ...
    'videoFile', 'three_quadrotor_formation.mp4');

cfg = mergeStruct(cfg, userCfg);

% 如果用户只修改了质量或重力而没有显式给出初始推力，自动把初始
% 推力更新为新的悬停值，避免启动时产生无关的竖直瞬态。
if ~isfield(userCfg, 'initial') || ...
        ~isfield(userCfg.initial, 'thrustNewton')
    cfg.initial.thrustNewton = cfg.vehicle.mass * cfg.vehicle.gravity;
end

% 允许用户用 N 个质量/初值；对单个矩阵输入做必要的形状规范化。
nVehicles = size(cfg.formation.offsets, 2);
cfg.vehicle.mass = reshape(cfg.vehicle.mass, [], 1);
if numel(cfg.vehicle.mass) == 1
    cfg.vehicle.mass = repmat(cfg.vehicle.mass, nVehicles, 1);
end
cfg.vehicle.maxTotalThrust = reshape(cfg.vehicle.maxTotalThrust, [], 1);
if numel(cfg.vehicle.maxTotalThrust) == 1
    cfg.vehicle.maxTotalThrust = repmat(cfg.vehicle.maxTotalThrust, nVehicles, 1);
end
if size(cfg.vehicle.inertia, 3) == 1
    cfg.vehicle.inertia = repmat(cfg.vehicle.inertia, 1, 1, nVehicles);
end
if size(cfg.vehicle.externalForce, 2) == 1
    cfg.vehicle.externalForce = repmat(cfg.vehicle.externalForce, 1, nVehicles);
end
if numel(cfg.formation.bl) == 1
    cfg.formation.bl = repmat(cfg.formation.bl, nVehicles, 1);
else
    cfg.formation.bl = reshape(cfg.formation.bl, [], 1);
end
end

function out = mergeStruct(base, override)
%MERGESTRUCT 递归合并结构体，便于入口脚本只覆盖少量参数。
out = base;
fields = fieldnames(override);
for i = 1:numel(fields)
    name = fields{i};
    if isstruct(override.(name)) && isfield(base, name) && ...
            isstruct(base.(name))
        out.(name) = mergeStruct(base.(name), override.(name));
    else
        out.(name) = override.(name);
    end
end
end
