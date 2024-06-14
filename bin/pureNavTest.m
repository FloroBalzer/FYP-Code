clc; clear; close all
% Pro-Nav gain
N=2;
% attacker init prams
attacker.x = 5;
attacker.y = 10;
attacker.z = 10;
attacker.vx = 1;
attacker.vy = 1;
attacker.vz = 1;
% target init prams
target.anchored = false; % fix target to init pos when true
target.x = 0;
target.y = 0;
target.z = 0;
target.vx = 0;
target.vy = 1;
target.vz = 0;
% Sim params
S  = 2000;    % max sim duration, seconds
dt = 0.05;     % time-step size, seconds
iter = S/dt; % max iteration num
% Pre-allocate sim
sim.t = nan(1, iter);    % elapsed time
sim.ax = nan(1, iter);
sim.ay = nan(1, iter);
sim.az = nan(1, iter);
sim.avx = nan(1, iter);
sim.avy = nan(1, iter);
sim.avz = nan(1, iter);
sim.av = nan(1, iter);
sim.tx = nan(1, iter);
sim.ty = nan(1, iter);
sim.tz = nan(1, iter);
sim.tvx = nan(1, iter);
sim.tvy = nan(1, iter);
sim.tvz = nan(1, iter);
sim.tv = nan(1, iter);
sim.R = nan(1, iter);


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
%--------------------------------------------------------------------------
% Run sim
%--------------------------------------------------------------------------
for k = 1:iter

    %relative position in inertial frame
    RTP = [(target.x-attacker.x), (target.y-attacker.y),(target.z-attacker.z)];
    R = norm(RTP);

    %Line of sight unit vector
    LOSu = RTP/R;
    
    % Terminate sim at intercept
    if abs(R) <= 0.5
        disp('target hit');
        disp(sim.t(k-1))
        break;
    end
    
    if(~target.anchored)
        target.x = target.x+target.vx*dt;
        target.y = target.y+target.vy*dt;
        target.z = target.z+target.vz*dt;

        % v_x = target.vx +5*cos(0.001*k*pi)*dt;
        % target.vx = v_x/(norm([target.vx;target.vy;target.vz]));
        target.vx  =target.vx;
        target.vy = target.vy;
        target.vz = target.vz;
    end
        
    % attacker pos
    attacker.x = attacker.x+attacker.vx*dt;
    attacker.y = attacker.y+attacker.vy*dt;
    attacker.z = attacker.z+attacker.vz*dt;
        
    % update attacker velocities
    attacker.vx = N*LOSu(1);
    attacker.vy = N*LOSu(2);
    attacker.vz = N*LOSu(3);
    
    %log data
    sim.t(k)   = k*dt;
    sim.ax(k) = attacker.x;
    sim.ay(k) = attacker.y;
    sim.az(k) = attacker.z;
    sim.avx(k) = attacker.vx;
    sim.avy(k) = attacker.vy;
    sim.avz(k) = attacker.vz;
    sim.av(k) = norm([sim.avx(k) sim.avy(k) sim.avz(k)]);
    sim.tx(k) = target.x;
    sim.ty(k) = target.y;
    sim.tz(k) = target.z;
    sim.tvx(k) = target.vx;
    sim.tvy(k) = target.vy;
    sim.tvz(k) = target.vz;
    sim.tv(k) = norm([sim.tvx(k) sim.tvy(k) sim.tvz(k)]);
    sim.R(k) = R;
end
% visualize results
% sum(~isnan(sim.ax()))
% sum(~isnan(sim.ay()))
% sum(~isnan(sim.az()))
%sim.ax(1:sum(~isnan(sim.ax())))
% sim.ax
% attacker = [sim.ax; sim.ay; sim.az]
close all;
%
figure
plot(sim.t,sim.R);
ylabel('Range (m)')
xlabel('Elapsed time (sec)')

figure
hold on;
plot(sim.t, sim.av, 'LineWidth',3);
plot(sim.t, sim.tv, 'LineWidth',3);
plot(sim.t, sim.tvx, 'LineWidth',1.5, 'LineStyle','-');
plot(sim.t, sim.tvy, 'LineWidth',1.5, 'LineStyle','-');
plot(sim.t, sim.tvz, 'LineWidth',1.5, 'LineStyle','-');
plot(sim.t, sim.avx, 'LineWidth',1.5, 'LineStyle',':');
plot(sim.t, sim.avy, 'LineWidth',1.5, 'LineStyle',':');
plot(sim.t, sim.avz, 'LineWidth',1.5, 'LineStyle',':');
legend('Attacker', 'Target', 'Target vx', 'Target vy',' Target vz',...
    'Attacker vx', 'Attacker vy',' Attacker vz',...
    'Location', 'southoutside','Orientation','horizontal');
ylabel('velocity (m/s)')
xlabel('Elapsed time (s)')
hold off;

% Position
figure;
scatter3(sim.ax, sim.ay, sim.az, 'filled'); hold on;
scatter3(sim.tx, sim.ty, sim.tz, 'filled');
title(['Pure Navigation, N = ' num2str(N) ])
legend('Attacker', 'Target', 'Location', 'southoutside','Orientation','horizontal')
xlabel('X')
ylabel('Y')
zlabel('Z')