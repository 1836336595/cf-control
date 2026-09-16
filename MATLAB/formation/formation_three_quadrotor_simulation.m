function sim = formation_three_quadrotor_simulation(userCfg)
%FORMATION_THREE_QUADROTOR_SIMULATION 三架四旋翼的 CTBR 编队仿真主程序。
%
% 用法：
%   sim = formation_three_quadrotor_simulation();
%   cfg = formation_three_quadrotor_parameters();
%   cfg.formation.type = 'leader_follower';
%   cfg.visualization.plot = false;
%   cfg.visualization.animate = false;
%   sim = formation_three_quadrotor_simulation(cfg);
%
% 仿真流程与 simulator/CTBR/codex/v2 一致：
%   编队加速度 u_i -> 期望合力 A_i -> R_c,i/Omega_c,i
%   -> CTBR 推力百分比和机体角速度 -> 内部角速度环 -> 六自由度动力学。

if nargin < 1 || isempty(userCfg)
    userCfg = struct();
end

cfg = formation_three_quadrotor_parameters(userCfg);
N = size(cfg.formation.offsets, 2);
dt = cfg.simulation.dt;
nSteps = floor(cfg.simulation.duration / dt) + 1;
time = (0:nSteps - 1) * dt;

% 初始化状态。数组的第二维是飞机编号，第三维是旋转矩阵页码。
position = cfg.initial.position;
velocity = cfg.initial.velocity;
rotation = cfg.initial.R;
bodyRate = cfg.initial.bodyRate;
actualThrust = cfg.initial.thrustNewton(:);

% 允许用户用一个共同的 3x1 初值，也允许直接给出 3xN 初值。
if numel(position) == 3
    position = repmat(position(:), 1, N);
else
    position = reshape(position, 3, N);
end
if numel(velocity) == 3
    velocity = repmat(velocity(:), 1, N);
else
    velocity = reshape(velocity, 3, N);
end
if numel(bodyRate) == 3
    bodyRate = repmat(bodyRate(:), 1, N);
else
    bodyRate = reshape(bodyRate, 3, N);
end
if numel(actualThrust) == 1
    actualThrust = repmat(actualThrust, N, 1);
elseif numel(actualThrust) ~= N
    error('formation:InitialThrustSize', ...
        'initial.thrustNewton 必须为标量或 N 元向量。');
end
if size(rotation, 3) == 1
    rotation = repmat(rotation, 1, 1, N);
end
for i = 1:N
    rotation(:, :, i) = projectSO3(rotation(:, :, i));
end

memory = initializeMemory(N);

% 日志数组。
sim = struct();
sim.time = time;
sim.position = zeros(3, N, nSteps);
sim.velocity = zeros(3, N, nSteps);
sim.acceleration = zeros(3, N, nSteps);
sim.rotation = zeros(3, 3, N, nSteps);
sim.bodyRate = zeros(3, N, nSteps);
sim.targetPosition = zeros(3, N, nSteps);
sim.targetVelocity = zeros(3, N, nSteps);
sim.targetAcceleration = zeros(3, N, nSteps);
sim.targetRotation = zeros(3, 3, N, nSteps);
sim.computedRotation = zeros(3, 3, N, nSteps);
sim.positionError = zeros(3, N, nSteps);
sim.velocityError = zeros(3, N, nSteps);
sim.formationErrorNorm = zeros(N, nSteps);
sim.leaderErrorNorm = zeros(N, nSteps);
sim.computedBodyRate = zeros(3, N, nSteps);
sim.bodyRateCommand = zeros(3, N, nSteps);
sim.bodyRateCommandDeg = zeros(3, N, nSteps);
sim.thrustNewtonCommand = zeros(N, nSteps);
sim.thrustNewtonActual = zeros(N, nSteps);
sim.thrustPercentage = zeros(N, nSteps);
sim.innerMoment = zeros(3, N, nSteps);
sim.formationAcceleration = zeros(3, N, nSteps);
sim.desiredForce = zeros(3, N, nSteps);
sim.desiredForceDot = zeros(3, N, nSteps);
sim.omegaCMethod = char(cfg.attitudeController.omegaCMethod);

for k = 1:nSteps
    t = time(k);
    reference = makeFormationReference(t, cfg, N);

    % 当前实际加速度用于 A_dot 的解析计算。这里使用上一个采样时刻的
    % 实际推力，与 V2 单机仿真的计算顺序相同。
    acceleration = zeros(3, N);
    for i = 1:N
        externalForce = cfg.vehicle.externalForce(:, min(i, size(cfg.vehicle.externalForce, 2)));
        acceleration(:, i) = cfg.vehicle.gravity * [0; 0; 1] ...
            - (actualThrust(i) / cfg.vehicle.mass(i)) * ...
            (rotation(:, :, i) * [0; 0; 1]) ...
            + externalForce / cfg.vehicle.mass(i);
    end

    states = struct( ...
        'position', position, ...
        'velocity', velocity, ...
        'acceleration', acceleration, ...
        'rotation', rotation, ...
        'bodyRate', bodyRate);

    % 先一次性得到所有飞机的编队控制量，再一次性构造所有 R_c。
    [command, memory] = formation_three_quadrotor_controller( ...
        states, reference, memory, cfg);

    % 记录当前状态和控制器输出。
    sim.position(:, :, k) = position;
    sim.velocity(:, :, k) = velocity;
    sim.acceleration(:, :, k) = acceleration;
    sim.rotation(:, :, :, k) = rotation;
    sim.bodyRate(:, :, k) = bodyRate;
    sim.targetPosition(:, :, k) = reference.position;
    sim.targetVelocity(:, :, k) = reference.velocity;
    sim.targetAcceleration(:, :, k) = reference.acceleration;
    sim.targetRotation(:, :, :, k) = reference.rotation;
    sim.computedRotation(:, :, :, k) = command.computedRotation;
    sim.positionError(:, :, k) = command.positionError;
    sim.velocityError(:, :, k) = command.velocityError;
    sim.formationErrorNorm(:, k) = command.formationErrorNorm;
    sim.leaderErrorNorm(:, k) = command.leaderErrorNorm;
    sim.computedBodyRate(:, :, k) = command.computedBodyRate;
    sim.bodyRateCommand(:, :, k) = command.bodyRateCommand;
    sim.bodyRateCommandDeg(:, :, k) = command.bodyRateCommandDeg;
    sim.thrustNewtonCommand(:, k) = command.thrustNewton;
    sim.thrustNewtonActual(:, k) = actualThrust;
    sim.thrustPercentage(:, k) = command.thrustPercentage;
    sim.formationAcceleration(:, :, k) = command.formationAcceleration;
    sim.desiredForce(:, :, k) = command.desiredForce;
    sim.desiredForceDot(:, :, k) = command.desiredForceDot;

    if k == nSteps
        break;
    end

    % -------- 模拟 CTBR 推力执行器 --------
    requestedThrust = cfg.vehicle.maxTotalThrust .* ...
        command.thrustPercentage / 100;
    thrustAlpha = min(1, dt / max(cfg.vehicle.thrustTimeConstant, eps));
    actualThrust = actualThrust + thrustAlpha * ...
        (requestedThrust - actualThrust);
    actualThrust = clampVector(actualThrust, 0, cfg.vehicle.maxTotalThrust);

    % -------- 模拟 Crazyflie 内部角速度 PID --------
    rateError = command.bodyRateCommand - bodyRate;
    memory.rateIntegral = memory.rateIntegral + dt * rateError;
    memory.rateIntegral = clampVector(memory.rateIntegral, ...
        -repmat(cfg.rateController.integralLimit, 1, N), ...
         repmat(cfg.rateController.integralLimit, 1, N));

    commandedAngularAcceleration = repmat(cfg.rateController.kp, 1, N) .* ...
        rateError + repmat(cfg.rateController.ki, 1, N) .* memory.rateIntegral;
    moment = zeros(3, N);
    bodyRateDot = zeros(3, N);
    for i = 1:N
        J = cfg.vehicle.inertia(:, :, i);
        omega = bodyRate(:, i);
        moment(:, i) = J * commandedAngularAcceleration(:, i) + ...
            cross(omega, J * omega);
        moment(:, i) = clampVector(moment(:, i), ...
            -cfg.rateController.maxMoment, cfg.rateController.maxMoment);
        bodyRateDot(:, i) = J \ (moment(:, i) - cross(omega, J * omega));
    end
    sim.innerMoment(:, :, k) = moment;

    % -------- 六自由度刚体动力学 --------
    velocity = velocity + dt * acceleration;
    position = position + dt * velocity;
    bodyRate = bodyRate + dt * bodyRateDot;
    bodyRate = clampVector(bodyRate, ...
        -repmat(cfg.simulation.maxBodyRate, 1, N), ...
         repmat(cfg.simulation.maxBodyRate, 1, N));
    for i = 1:N
        rotation(:, :, i) = projectSO3(rotation(:, :, i) * ...
            expSO3(bodyRate(:, i) * dt));
    end
end

sim.config = cfg;
sim.lastCommand = command;

if cfg.visualization.plot || cfg.visualization.animate
    formation_three_quadrotor_visualization(sim, cfg);
end
end

function memory = initializeMemory(N)
%INITIALIZEMEMORY 控制器跨采样周期保存的状态。
memory = struct();
memory.positionIntegral = zeros(3, N);
memory.attitudeIntegral = zeros(3, N);
memory.rateIntegral = zeros(3, N);
memory.previousComputedRotation = repmat(eye(3), 1, 1, N);
memory.previousB1 = repmat([1; 0; 0], 1, N);
memory.previousB3 = repmat([0; 0; 1], 1, N);
memory.hasPreviousRotation = false(1, N);
end

function reference = makeFormationReference(t, cfg, N)
%MAKEFORMATIONREFERENCE 把共同 8 字轨迹扩展为每架飞机的参考状态。
% callReferenceFunction 遵循 V2 的六输出顺序：位置、速度、加速度、
% 姿态、jerk、姿态导数。
[pc, vc, ac, Rd, jerkc, RdDot] = callReferenceFunction(cfg.referenceFcn, t);
offsets = cfg.formation.offsets;
reference = struct();
reference.centerPosition = pc(:);
reference.centerVelocity = vc(:);
reference.centerAcceleration = ac(:);
reference.centerJerk = jerkc(:);
reference.position = repmat(pc(:), 1, N) + offsets;
reference.velocity = repmat(vc(:), 1, N);
reference.acceleration = repmat(ac(:), 1, N);
reference.jerk = repmat(jerkc(:), 1, N);
reference.rotation = repmat(projectSO3(Rd), 1, 1, N);
reference.rotationDot = repmat(RdDot, 1, 1, N);
end

function [pc, vc, ac, Rd, jerkc, RdDot] = callReferenceFunction(referenceFcn, t)
%CALLREFERENCEFUNCTION 兼容 V2 六输出、五输出和旧式四输出轨迹函数。
numberOfOutputs = nargout(referenceFcn);
if numberOfOutputs >= 6
    % V2 顺序为 [p,v,a,R,jerk,Rdot]。早期本目录版本曾使用
    % [p,v,a,jerk,R,Rdot]，这里按第四、第五输出的尺寸自动兼容。
    [pc, vc, ac, fourth, fifth, RdDot] = referenceFcn(t);
    if isequal(size(fourth), [3, 3])
        Rd = fourth;
        jerkc = fifth;
    elseif isequal(size(fifth), [3, 3])
        jerkc = fourth;
        Rd = fifth;
    else
        error('formation:ReferenceOutputSize', ...
            '六输出轨迹函数的第四或第五输出必须是 3x3 姿态矩阵。');
    end
elseif numberOfOutputs == 5
    [pc, vc, ac, fourth, fifth] = referenceFcn(t);
    if isequal(size(fourth), [3, 3])
        Rd = fourth;
        jerkc = fifth;
    elseif isequal(size(fifth), [3, 3])
        jerkc = fourth;
        Rd = fifth;
    else
        error('formation:ReferenceOutputSize', ...
            '五输出轨迹函数的第四或第五输出必须是 3x3 姿态矩阵。');
    end
    RdDot = zeros(3, 3);
elseif numberOfOutputs == 4
    [pc, vc, ac, Rd] = referenceFcn(t);
    jerkc = zeros(3, 1);
    RdDot = zeros(3, 3);
else
    % 对匿名函数或 varargout 函数，先尝试新接口，失败后回退到四输出。
    try
        [pc, vc, ac, fourth, fifth, RdDot] = referenceFcn(t);
        if isequal(size(fourth), [3, 3])
            Rd = fourth;
            jerkc = fifth;
        elseif isequal(size(fifth), [3, 3])
            jerkc = fourth;
            Rd = fifth;
        else
            error('formation:ReferenceOutputSize', ...
                '六输出轨迹函数的第四或第五输出必须是 3x3 姿态矩阵。');
        end
    catch
        [pc, vc, ac, Rd] = referenceFcn(t);
        jerkc = zeros(3, 1);
        RdDot = zeros(3, 3);
    end
end
end

function R = projectSO3(R)
%PROJECTSO3 把数值误差造成的近似旋转矩阵投影回 SO(3)。
[U, ~, V] = svd(R);
R = U * V';
if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end
end

function R = expSO3(v)
%EXPSO3 SO(3) 指数映射，用于离散更新姿态。
theta = norm(v);
if theta < 1e-10
    R = eye(3) + hatMap(v);
else
    K = hatMap(v / theta);
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K * K;
end
end

function S = hatMap(v)
%HATMAP 三维向量到反对称矩阵的 hat 映射。
S = [0, -v(3), v(2); ...
     v(3), 0, -v(1); ...
     -v(2), v(1), 0];
end

function value = clampVector(value, lowerBound, upperBound)
%CLAMPVECTOR 对标量、列向量或矩阵逐元素限幅。
value = min(max(value, lowerBound), upperBound);
end
