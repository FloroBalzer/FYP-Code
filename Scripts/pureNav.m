function [AttackerDataLog, TargetDataLog, MiscDataLog] = pureNav(param, attackerParam, N)
%pureNav: generate Attacker and Target trajectories based on simulation
%initial conditions and Gain N

% attacker init prams
attacker.x = attackerParam.x0;
attacker.y = attackerParam.y0;
attacker.z = attackerParam.z0;
attacker.vx = attackerParam.vx0;
attacker.vy = attackerParam.vy0;
attacker.vz = attackerParam.vz0;
% target init prams
target.x = param.target.x0;
target.y = param.target.y0;
target.z = param.target.z0;
target.vx = param.target.vx0;
target.vy = param.target.vy0;
target.vz = param.target.vz0;

%Log
S  = param.Tt;    % max sim duration, seconds
dt = param.Ts;     % time-step size, seconds
iter = S/dt; % max iteration num
hitDistance = param.hitDistance; %min distance required to register hit
hitflag=0;
% Pre-allocate sim

AttackerDataLog.t = nan(1, iter);
AttackerDataLog.x = nan(1, iter);
AttackerDataLog.y = nan(1, iter);
AttackerDataLog.z = nan(1, iter);
AttackerDataLog.vx = nan(1, iter);
AttackerDataLog.vy = nan(1, iter);
AttackerDataLog.vz = nan(1, iter);
AttackerDataLog.v = nan(1, iter);

TargetDataLog.t = nan(1, iter);
TargetDataLog.x = nan(1, iter);
TargetDataLog.y = nan(1, iter);
TargetDataLog.z = nan(1, iter);
TargetDataLog.vx = nan(1, iter);
TargetDataLog.vy = nan(1, iter);
TargetDataLog.vz = nan(1, iter);
TargetDataLog.v = nan(1, iter);

MiscDataLog.R = nan(1, iter);
MiscDataLog.hitTime =0;
MiscDataLog.hitDistance =0;
MiscDataLog.hitPosition = 0;

%--------------------------------------------------------------------------
% Init sim
%--------------------------------------------------------------------------
% Target pos
target.x = target.x+target.vx*dt;
target.y = target.y+target.vy*dt;
target.z = target.z+target.vz*dt;
% attacker pos
attacker.x = attacker.x+attacker.vx*dt;
attacker.y = attacker.y+attacker.vy*dt;
attacker.z = attacker.z+attacker.vz*dt;
vn = norm([attacker.vx attacker.vy attacker.vz]);

%--------------------------------------------------------------------------
% Run sim
%--------------------------------------------------------------------------
for k = 1:iter

    %relative position in inertial frame
    RTA = [(target.x-attacker.x), (target.y-attacker.y),(target.z-attacker.z)];
    R = norm(RTA);

    %Line of sight unit vector
    LOSu = RTA/R;
    LOSu = LOSu*vn;
    
    % Terminate sim at intercept
    if abs(R) <= hitDistance
        MiscDataLog.hitTime =k*dt;
        MiscDataLog.hitDistance = abs(R);
        MiscDataLog.hitPosition = [attacker.x attacker.y attacker.z];
        MiscDataLog.hitVelocity = [attacker.vx attacker.vy attacker.vz];
        hitflag=1;
        break;
    end
    
    %update target pos
    target.x = target.x+target.vx*dt;
    target.y = target.y+target.vy*dt;
    target.z = target.z+target.vz*dt;

    %update target velocities
    target.vx = target.vx;
    target.vy = target.vy;
    target.vz = target.vz;
   
        
    % attacker pos
    attacker.x = attacker.x+attacker.vx*dt;
    attacker.y = attacker.y+attacker.vy*dt;
    attacker.z = attacker.z+attacker.vz*dt;
        
    % update attacker velocities
    attacker.vx = N*LOSu(1);
    attacker.vy = N*LOSu(2);
    attacker.vz = N*LOSu(3);
    
    %log data
    AttackerDataLog.t(k)   = k*dt;
    AttackerDataLog.x(k) = attacker.x;
    AttackerDataLog.y(k) = attacker.y;
    AttackerDataLog.z(k) = attacker.z;
    AttackerDataLog.vx(k) = attacker.vx;
    AttackerDataLog.vy(k) = attacker.vy;
    AttackerDataLog.vz(k) = attacker.vz;
    AttackerDataLog.v(k)= norm([attacker.vx attacker.vy attacker.vz]);

    TargetDataLog.t(k)   = k*dt;
    TargetDataLog.x(k) = target.x;
    TargetDataLog.y(k) = target.y;
    TargetDataLog.z(k) = target.z;
    TargetDataLog.vx(k) = target.vx;
    TargetDataLog.vy(k) = target.vy;
    TargetDataLog.vz(k) = target.vz;
    TargetDataLog.v(k)= norm([target.vx target.vy target.vz]);

    MiscDataLog.R(k) = R;
end
%remove NaN entries
% AttackerDataLog.t = AttackerDataLog.t(~isnan(AttackerDataLog.t));
% AttackerDataLog.x = AttackerDataLog.x(~isnan(AttackerDataLog.x));
% AttackerDataLog.y = AttackerDataLog.y(~isnan(AttackerDataLog.y));
% AttackerDataLog.z = AttackerDataLog.z(~isnan(AttackerDataLog.z));
% AttackerDataLog.vx = AttackerDataLog.vx(~isnan(AttackerDataLog.vx));
% AttackerDataLog.vy = AttackerDataLog.vy(~isnan(AttackerDataLog.vy));
% AttackerDataLog.vz = AttackerDataLog.vz(~isnan(AttackerDataLog.vz));
% AttackerDataLog.v= AttackerDataLog.v(~isnan(AttackerDataLog.v));
% 
% TargetDataLog.t = TargetDataLog.t(~isnan(TargetDataLog.t));
% TargetDataLog.x = TargetDataLog.x(~isnan(TargetDataLog.x));
% TargetDataLog.y = TargetDataLog.y(~isnan(TargetDataLog.y));
% TargetDataLog.z = TargetDataLog.z(~isnan(TargetDataLog.z));
% TargetDataLog.vx = TargetDataLog.vx(~isnan(TargetDataLog.vx));
% TargetDataLog.vy = TargetDataLog.vy(~isnan(TargetDataLog.vy));
% TargetDataLog.vz = TargetDataLog.vz(~isnan(TargetDataLog.vz));
% TargetDataLog.v= TargetDataLog.v(~isnan(TargetDataLog.v));
% 
% MiscDataLog.R= MiscDataLog.R(~isnan(MiscDataLog.R));

if hitflag==0
    MiscDataLog.hitTime =S;
    MiscDataLog.hitDistance = abs(R);
    MiscDataLog.hitPosition = [attacker.x attacker.y attacker.z];
    MiscDataLog.hitVelocity = [attacker.vx attacker.vy attacker.vz];
end

end