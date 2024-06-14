function [AttackerDataLog, TargetDataLog, MiscDataLog] = proNav(param, attackerParam, n)

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

initialDistance = sqrt((attacker.x - target.x)^2+(attacker.y - target.y)^2+(attacker.z - target.z)^2);
% Pro-Nav gain
N = initialDistance*n;
% Sim params
%Log
S  = param.Tt;    % max sim duration, seconds
dt = param.Ts;     % time-step size, seconds
iter = S/dt; % max iteration num
hitDistance = param.hitDistance;
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
% MiscDataLog.ZEM = nan(iter, 3);
% MiscDataLog.ZEMn = nan(iter, 3);

%--------------------------------------------------------------------------
% Init sim
%--------------------------------------------------------------------------
RTP_last = [(target.x-attacker.x), (target.y-attacker.y),(target.z-attacker.z)];
R_prev = norm(RTP_last);
LOSu = RTP_last/R_prev;
vn = norm([attacker.vx attacker.vy attacker.vz]);
tf = S;
% attacker initial velocity LOS
attacker.vx = target.vx + LOSu(1)*vn;
attacker.vy = target.vy + LOSu(2)*vn;
attacker.vz = target.vz + LOSu(3)*vn;


% Target pos
target.x = target.x+target.vx*dt;
target.y = target.y+target.vy*dt;
target.z = target.z+target.vz*dt;
% attacker pos
attacker.x = attacker.x+attacker.vx*dt;
attacker.y = attacker.y+attacker.vy*dt;
attacker.z = attacker.z+attacker.vz*dt;
%--------------------------------------------------------------------------
% Run sim
%--------------------------------------------------------------------------
for k = 1:iter
     %time to go
    tgo = tf - k*dt;

    %relative position in inertial frame
    RTP = [(target.x-attacker.x), (target.y-attacker.y),(target.z-attacker.z)];
    R = norm(RTP);

    %relative velocity in inertial frame
    VTP = [(target.vx-attacker.vx), (target.vy-attacker.vy),(target.vz-attacker.vz)];

    %Zero Effort Miss at time step
    ZEM = (RTP+(VTP*tgo));

    %Line of sight unit vector
    LOSu = RTP/R;

    %ZEM normal to LOS
    ZEMn = ZEM  - (dot(ZEM,LOSu)*LOSu);
    
    % ProNav with zero effort miss
    a1 = (N * ZEMn(1))/tgo^2;
    a2 = (N * ZEMn(2))/tgo^2;
    a3 = (N * ZEMn(3))/tgo^2;
    
    if abs(R) <= hitDistance %|| abs(R)>10*initialDistance
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


    %update attacker pos
    attacker.x = attacker.x+attacker.vx*dt;
    attacker.y = attacker.y+attacker.vy*dt;
    attacker.z = attacker.z+attacker.vz*dt;
    
    %update attacker velocities
    attacker.vx = attacker.vx +a1*dt;
    attacker.vy = attacker.vy +a2*dt;
    attacker.vz = attacker.vz +a3*dt;
    % else
    %     %update attacker pos
    %     attacker.x = attacker.x+attacker.vx*dt;
    %     attacker.y = attacker.y+attacker.vy*dt;
    %     attacker.z = attacker.z+attacker.vz*dt;
    % 
    %     % update attacker velocities
    %     attacker.vx = n*vn*LOSu(1);
    %     attacker.vy = n*vn*LOSu(2);
    %     attacker.vz = n*vn*LOSu(3);
    % end

    RTP_last = RTP;
    R_prev=R;

    %log Data
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
    % MiscDataLog.ZEM(k,:) = ZEM;
    % MiscDataLog.ZEMn(k,:) = ZEMn;
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