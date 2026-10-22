function [command, memory] = formation_three_quadrotor_controller( ...
    states, reference, memory, cfg)
%FORMATION_THREE_QUADROTOR_CONTROLLER 三机编队几何 CTBR 外环。
%
% 该文件把编队论文中的二阶控制律和几何控制单机.pdf/V2 中的姿态
% 构造串起来。它不直接输出力矩，而是输出：
%   1) 每架飞机的期望总推力（再映射为百分比）；
%   2) 每架飞机的期望机体角速度。
%
% 关键链路：编队控制律得到 u_i -> A_i=-m_i(g*e3-u_i)
% -> b3c_i -> R_c,i -> Omega_c,i。
% displacement 模式中，1 号机采用几何位置控制；其余飞机采用相对
% 位移一致性控制。displacement_pure 模式可复现原始纯相对位移控制。

N = size(states.position, 2);
e3 = [0; 0; 1];
I3 = eye(3);

u = zeros(3, N);
uDot = zeros(3, N);
formationErrorNorm = zeros(N, 1);
leaderErrorNorm = zeros(N, 1);
relativePositionError = zeros(3, N, N);
relativeVelocityError = zeros(3, N, N);

% ---------- 第一步：同时计算所有飞机的编队加速度 u_i ----------
for i = 1:N
    [E, V, eNormSquared] = relativeSums(i, states, reference, cfg);
    relativePositionError(:, i, :) = ...
        reshape(states.position(:, i) - states.position, 3, 1, N) - ...
        reshape(cfg.formation.offsets(:, i) - cfg.formation.offsets, 3, 1, N);
    relativeVelocityError(:, i, :) = ...
        reshape(states.velocity(:, i) - states.velocity, 3, 1, N);
    formationErrorNorm(i) = sqrt(max(eNormSquared, 0));

    switch lower(char(cfg.formation.type))
        case 'displacement'
            leaderIndex = cfg.formation.displacementLeaderIndex;
            if i == leaderIndex
                % 几何控制单机的位置外环：1 号机直接跟踪自己的 8 字
                % 参考点，后续再统一映射到 A、Rc 和 Omega_c。
                ex = states.position(:, i) - reference.position(:, i);
                ev = states.velocity(:, i) - reference.velocity(:, i);
                integralRate = ev + cfg.geometricController.c1 * ex;
                memory.positionIntegral(:, i) = memory.positionIntegral(:, i) ...
                    + cfg.simulation.dt * integralRate;
                memory.positionIntegral(:, i) = clampVector( ...
                    memory.positionIntegral(:, i), ...
                    -cfg.geometricController.integralLimit, ...
                     cfg.geometricController.integralLimit);
                u(:, i) = reference.centerAcceleration ...
                    - cfg.geometricController.kx * ex ...
                    - cfg.geometricController.kv * ev ...
                    - cfg.geometricController.ki * memory.positionIntegral(:, i);
                leaderErrorNorm(i) = norm(ex);
            else
                % 2/3 号机不使用绝对位置误差，只使用邻居相对位置和
                % 相对速度；共同加速度是让整个相对编队跟随 8 字的前馈。
                u(:, i) = -cfg.formation.kp * E - cfg.formation.kv * V;
                if cfg.formation.commonAcceleration
                    u(:, i) = u(:, i) + reference.centerAcceleration;
                end
                leaderErrorNorm(i) = norm(states.position(:, i) - ...
                    reference.position(:, i));
            end

        case 'displacement_pure'
            % 纯相对位移控制：所有飞机都不使用绝对位置误差。
            u(:, i) = -cfg.formation.kp * E - cfg.formation.kv * V;
            if cfg.formation.commonAcceleration
                u(:, i) = u(:, i) + reference.centerAcceleration;
            end
            leaderErrorNorm(i) = norm(states.position(:, i) - ...
                reference.position(:, i));

        case {'leader_follower', 'leader-follower'}
            eLeader = states.position(:, i) - reference.position(:, i);
            eta = states.velocity(:, i) - reference.velocity(:, i);
            b = cfg.formation.bl(i);
            % 虚拟领导者是编队中心点；bl(i)>0 的每架飞机都直接获得
            % 中心点误差，其余项仍完全由通信图上的相对信息组成。
            u(:, i) = reference.centerAcceleration ...
                - cfg.formation.kf * E ...
                - cfg.formation.kvf * V ...
                - cfg.formation.kbl * b * eLeader ...
                - cfg.formation.kvl * b * eta;
            leaderErrorNorm(i) = norm(eLeader);

        otherwise
            error('formation:UnknownType', ...
                'cfg.formation.type 必须是 displacement、displacement_pure 或 leader_follower。');
    end
end

% ---------- 第二步：计算 A_dot，供解析 Omega_c 使用 ----------
for i = 1:N
    [~, V, ~] = relativeSums(i, states, reference, cfg);
    accelerationDifference = zeros(3, 1);
    for j = 1:N
        aij = cfg.formation.adjacency(i, j);
        if j ~= i && aij ~= 0
            accelerationDifference = accelerationDifference + aij * ...
                (states.acceleration(:, i) - states.acceleration(:, j));
        end
    end

    switch lower(char(cfg.formation.type))
        case 'displacement'
            leaderIndex = cfg.formation.displacementLeaderIndex;
            if i == leaderIndex
                ex = states.position(:, i) - reference.position(:, i);
                ev = states.velocity(:, i) - reference.velocity(:, i);
                integralRate = ev + cfg.geometricController.c1 * ex;
                integralDerivative = saturatedIntegralDerivative( ...
                    memory.positionIntegral(:, i), integralRate, ...
                    cfg.geometricController.integralLimit);
                uDot(:, i) = reference.centerJerk ...
                    - cfg.geometricController.kx * ev ...
                    - cfg.geometricController.kv * ...
                      (states.acceleration(:, i) - reference.centerAcceleration) ...
                    - cfg.geometricController.ki * integralDerivative;
            else
                uDot(:, i) = -cfg.formation.kp * V ...
                    - cfg.formation.kv * accelerationDifference;
                if cfg.formation.commonAcceleration
                    uDot(:, i) = uDot(:, i) + reference.centerJerk;
                end
            end

        case 'displacement_pure'
            uDot(:, i) = -cfg.formation.kp * V ...
                - cfg.formation.kv * accelerationDifference;
            if cfg.formation.commonAcceleration
                uDot(:, i) = uDot(:, i) + reference.centerJerk;
            end

        case {'leader_follower', 'leader-follower'}
            eta = states.velocity(:, i) - reference.velocity(:, i);
            etaDot = states.acceleration(:, i) - reference.centerAcceleration;
            b = cfg.formation.bl(i);
            uDot(:, i) = reference.centerJerk ...
                - cfg.formation.kf * V ...
                - cfg.formation.kvf * accelerationDifference ...
                - cfg.formation.kbl * b * eta ...
                - cfg.formation.kvl * b * etaDot;
    end
end

% ---------- 第三步：从 A_i 构造 R_c,i、Omega_c,i 和 CTBR 命令 ----------
computedRotation = zeros(3, 3, N);
computedBodyRate = zeros(3, N);
bodyRateCommand = zeros(3, N);
attitudeError = zeros(3, N);
thrustNewton = zeros(N, 1);
thrustPercentage = zeros(N, 1);
desiredForce = zeros(3, N);
desiredForceDot = zeros(3, N);

for i = 1:N
    mass = cfg.vehicle.mass(i);
    A = -mass * (cfg.vehicle.gravity * e3 - u(:, i));
    ADot = mass * uDot(:, i);
    desiredForce(:, i) = A;
    desiredForceDot(:, i) = ADot;

    forceNorm = norm(A);
    if forceNorm < cfg.omegaC.forceNormEpsilon
        % 合力接近零时方向没有定义，沿用上一帧 b3c。
        b3c = memory.previousB3(:, i);
        forceNorm = 0;
    else
        b3c = -A / forceNorm;
    end

    b1d = reference.rotation(:, 1, i);
    b1dDot = reference.rotationDot(:, 1, i);
    P = I3 - b3c * b3c';
    projection = P * b1d;
    projectionNorm = norm(projection);
    if projectionNorm < cfg.omegaC.headingProjectionEpsilon
        % 航向轴与推力轴平行时，使用上一帧航向，并重新投影。
        projection = P * memory.previousB1(:, i);
        projectionNorm = norm(projection);
        if projectionNorm < cfg.omegaC.headingProjectionEpsilon
            % 选择与 b3c 最不平行的坐标轴，避免 b3c 恰好沿 e1
            % 时固定备用方向仍然无法投影的问题。
            [~, fallbackIndex] = min(abs(b3c));
            fallbackAxis = I3(:, fallbackIndex);
            projection = P * fallbackAxis;
            projectionNorm = norm(projection);
        end
        b1dDot = zeros(3, 1);
    end
    b1c = projection / max(projectionNorm, eps);
    b2c = cross(b3c, b1c);
    b2c = b2c / max(norm(b2c), eps);
    % 再正交化，抑制有限精度造成的漂移。
    b1c = cross(b2c, b3c);
    b1c = b1c / max(norm(b1c), eps);
    Rc = projectSO3([b1c, b2c, b3c]);

    omegaCMethod = lower(char(cfg.attitudeController.omegaCMethod));
    switch omegaCMethod
        case {'analytic', 'derivative'}
            if forceNorm == 0
                omegaC = zeros(3, 1);
            else
                omegaC = analyticOmegaC(Rc, A, ADot, b1d, b1dDot, ...
                    cfg.omegaC.forceNormEpsilon, ...
                    cfg.omegaC.headingProjectionEpsilon);
            end
        case {'log_difference', 'log', 'so3_log'}
            if memory.hasPreviousRotation(i)
                relativeRotation = memory.previousComputedRotation(:, :, i)' * Rc;
                omegaC = so3Log(relativeRotation) / max(cfg.simulation.dt, eps);
            else
                omegaC = zeros(3, 1);
            end
        otherwise
            error('formation:UnknownOmegaCMethod', ...
                '未知 Omega_c 方法：%s。可选 analytic 或 log_difference。', ...
                cfg.attitudeController.omegaCMethod);
    end

    R = states.rotation(:, :, i);
    eR = 0.5 * veeMap(Rc' * R - R' * Rc);
    if cfg.attitudeController.useIntegral
        memory.attitudeIntegral(:, i) = memory.attitudeIntegral(:, i) ...
            + cfg.simulation.dt * eR;
        memory.attitudeIntegral(:, i) = clampVector(...
            memory.attitudeIntegral(:, i), ...
            -cfg.attitudeController.integralLimit, ...
             cfg.attitudeController.integralLimit);
    else
        memory.attitudeIntegral(:, i) = zeros(3, 1);
    end

    bodyRateCurrentFrame = R' * Rc * omegaC;
    commandRate = bodyRateCurrentFrame ...
        - cfg.attitudeController.kr .* eR ...
        - cfg.attitudeController.ki .* memory.attitudeIntegral(:, i);
    commandRate = clampVector(commandRate, ...
        -cfg.attitudeController.maxBodyRateCommand, ...
         cfg.attitudeController.maxBodyRateCommand);

    thrustNewton(i) = -A' * R * e3;
    thrustPercentage(i) = 100 * max(thrustNewton(i), 0) / ...
        cfg.vehicle.maxTotalThrust(i);
    thrustPercentage(i) = min(max(thrustPercentage(i), 0), 100);

    computedRotation(:, :, i) = Rc;
    computedBodyRate(:, i) = omegaC;
    attitudeError(:, i) = eR;
    bodyRateCommand(:, i) = commandRate;

    memory.previousComputedRotation(:, :, i) = Rc;
    memory.previousB1(:, i) = Rc(:, 1);
    memory.previousB3(:, i) = Rc(:, 3);
    memory.hasPreviousRotation(i) = true;
end

command = struct();
command.positionError = states.position - reference.position;
command.velocityError = states.velocity - reference.velocity;
command.relativePositionError = relativePositionError;
command.relativeVelocityError = relativeVelocityError;
command.formationErrorNorm = formationErrorNorm;
command.leaderErrorNorm = leaderErrorNorm;
command.formationAcceleration = u;
command.formationAccelerationDot = uDot;
command.computedRotation = computedRotation;
command.computedBodyRate = computedBodyRate;
command.bodyRateCommand = bodyRateCommand;
command.bodyRateCommandDeg = rad2deg(bodyRateCommand);
command.attitudeError = attitudeError;
command.thrustNewton = thrustNewton;
command.thrustPercentage = thrustPercentage;
command.desiredForce = desiredForce;
command.desiredForceDot = desiredForceDot;
command.omegaCMethod = char(cfg.attitudeController.omegaCMethod);
end

function [E, V, eNormSquared] = relativeSums(i, states, reference, cfg)
%RELATIVESUMS 计算 E_i=sum(a_ij e_ij) 和 V_i=sum(a_ij(v_i-v_j))。
N = size(states.position, 2);
E = zeros(3, 1);
V = zeros(3, 1);
eNormSquared = 0;
for j = 1:N
    aij = cfg.formation.adjacency(i, j);
    if j == i || aij == 0
        continue;
    end
    eij = (states.position(:, i) - states.position(:, j)) ...
        - (cfg.formation.offsets(:, i) - cfg.formation.offsets(:, j));
    vij = states.velocity(:, i) - states.velocity(:, j);
    E = E + aij * eij;
    V = V + aij * vij;
    eNormSquared = eNormSquared + aij * (eij' * eij);
end
% reference is intentionally an input here: it documents that e_ij is a
% relative error and does not use the absolute reference trajectory.
if isempty(reference)
    error('formation:InternalReferenceError', '缺少编队参考状态。');
end
end

function omegaC = analyticOmegaC(Rc, A, ADot, b1d, b1dDot, ...
    forceNormEpsilon, headingProjectionEpsilon)
%ANALYTICOMEGAC 按几何控制单机.pdf 和 V2 的列向量导数求 Omega_c。
b3c = Rc(:, 3);
forceNorm = norm(A);
if forceNorm < forceNormEpsilon
    omegaC = zeros(3, 1);
    return;
end

b3cDot = -(eye(3) - b3c * b3c') * ADot / forceNorm;
P = eye(3) - b3c * b3c';
projection = P * b1d;
projectionNorm = norm(projection);
if projectionNorm < headingProjectionEpsilon
    b1cDot = zeros(3, 1);
else
    projectionDot = -(b3cDot * b3c' + b3c * b3cDot') * b1d ...
        + P * b1dDot;
    b1c = Rc(:, 1);
    b1cDot = (eye(3) - b1c * b1c') * projectionDot / projectionNorm;
end

b1c = Rc(:, 1);
b2cDot = cross(b3cDot, b1c) + cross(b3c, b1cDot);
RcDot = [b1cDot, b2cDot, b3cDot];
OmegaHat = Rc' * RcDot;
OmegaHat = 0.5 * (OmegaHat - OmegaHat');
omegaC = veeMap(OmegaHat);
end

function v = so3Log(R)
%SO3LOG SO(3) 对数映射，返回旋转向量。
cosTheta = min(max((trace(R) - 1) / 2, -1), 1);
theta = acos(cosTheta);
if theta < 1e-7
    v = 0.5 * veeMap(R - R');
elseif abs(pi - theta) < 1e-5
    [V, D] = eig((R + eye(3)) / 2);
    [~, index] = max(real(diag(D)));
    axis = real(V(:, index));
    axis = axis / max(norm(axis), eps);
    v = theta * axis;
else
    v = theta / (2 * sin(theta)) * veeMap(R - R');
end
end

function v = veeMap(S)
%VEEMAP 反对称矩阵到三维向量的 vee 映射。
v = [S(3, 2); S(1, 3); S(2, 1)];
end

function R = projectSO3(R)
%PROJECTSO3 投影到合法旋转矩阵。
[U, ~, V] = svd(R);
R = U * V';
if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end
end

function value = clampVector(value, lowerBound, upperBound)
%CLAMPVECTOR 逐元素限幅。
value = min(max(value, lowerBound), upperBound);
end

function rate = saturatedIntegralDerivative(integralState, rawRate, limit)
%SATURATEDINTEGRALDERIVATIVE 饱和积分器的解析导数。
% 当积分状态已经到达边界且原始导数仍指向边界外时，sat 的导数为零。
rate = rawRate;
tolerance = 1e-12;
upperActive = integralState >= limit - tolerance & rawRate > 0;
lowerActive = integralState <= -limit + tolerance & rawRate < 0;
rate(upperActive | lowerActive) = 0;
end
