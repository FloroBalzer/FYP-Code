clc; clear; close all

% attacker init prams
attacker.x = 5;
attacker.y = 10;
attacker.z = 10;
attacker.vx = 1;
attacker.vy = 1;
attacker.vz = 1;
% target init prams
target.x = 0;
target.y = 0;
target.z = 0;
target.vx = 0;
target.vy = 1;
target.vz = 0;

initialDistance = sqrt((attacker.x - target.x)^2+(attacker.y - target.y)^2+(attacker.z - target.z)^2);
disp("initial distance:");
disp(initialDistance)
% Pro-Nav gain
n=2;
N = initialDistance*n;
% Sim params
S  = 2000;    % max sim duration, seconds
dt = 0.05;     % time-step size, seconds
iter = S/dt; % max iteration num
flag=0;
% Pre-allocate sim
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
sim.ZEMn = nan(iter,3);
sim.ZEM = nan(iter,3);

%--------------------------------------------------------------------------
% Init sim
%--------------------------------------------------------------------------
RTP_last = [(target.x-attacker.x), (target.y-attacker.y),(target.z-attacker.z)];
R_prev = norm(RTP_last);
LOSu = RTP_last/R_prev;
tf = S/(1/dt);
% attacker initial velocity LOS
attacker.vx = target.vx + LOSu(1);
attacker.vy = target.vy + LOSu(2);
attacker.vz = target.vz + LOSu(3);
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
    
    % Terminate sim at intercept
    % if flag ==1
    %     if abs(R)<abs(R_prev)
    %         flag=0;
    %         fprintf('flag reset at time %.2f\n', (k*dt));
    %     end
    % end
    
    % if abs(R)>abs(R_prev) && flag == 0
    %     fprintf('distance to target %.2f\n', (sqrt((sim.ax(k-2) - sim.tx(k-2))^2+((sim.ay(k-2) - sim.ty(k-2))^2+((sim.az(k-2) - sim.tz(k-2))^2)))));
    %     fprintf('time: %.2f\n', sim.t(k-1));
    %     flag=1;
    % end


    if abs(R)<0.5 || abs(R)>10*initialDistance
        disp('target hit');
        fprintf('time: %.2f\n', sim.t(k-1));
        % fprintf('distance to target %.2f\n', (sqrt((sim.ax(k-1) - sim.tx(k-1))^2+((sim.ay(k-1) - sim.ty(k-1))^2+((sim.az(k-1) - sim.tz(k-1))^2)))));
        break;
    end
    
 
    %update target pos
    target.x = target.x+target.vx*dt;
    target.y = target.y+target.vy*dt;
    target.z = target.z+target.vz*dt;
    
    %update target velocities
    % v_x = target.vx +5*cos(0.001*k*pi)*dt;
    % target.vx = v_x/(norm([target.vx;target.vy;target.vz]));
    target.vx = target.vx+dt*(cos(0.01*k));
    target.vy = target.vy+dt*(cos(0.01*k));
    target.vz = target.vz;

    %update attacker pos
    attacker.x = attacker.x+attacker.vx*dt;
    attacker.y = attacker.y+attacker.vy*dt;
    attacker.z = attacker.z+attacker.vz*dt;
    
    %update attacker velocities
    attacker.vx = attacker.vx +a1*dt;
    attacker.vy = attacker.vy +a2*dt;
    attacker.vz = attacker.vz +a3*dt;

    
    RTP_last = RTP;
    R_prev=R;
    
    %log data
    sim.t(k)   = k*dt;
    sim.ax(k) = attacker.x;
    sim.ay(k) = attacker.y;
    sim.az(k) = attacker.z;
    sim.avx(k) = attacker.vx;
    sim.avy(k) = attacker.vy;
    sim.avz(k) = attacker.vz;
    sim.av(k) = norm([attacker.vx attacker.vy attacker.vz]);
    sim.tx(k) = target.x;
    sim.ty(k) = target.y;
    sim.tz(k) = target.z;
    sim.tvx(k) = target.vx;
    sim.tvy(k) = target.vy;
    sim.tvz(k) = target.vz;
    sim.tv(k) = norm([target.vx target.vy target.vz]);
    sim.R(k) = R;
    sim.ZEMn(k,:) = ZEMn;
    sim.ZEM(k,:) = ZEM;
end
% visualize results
close all;
%
figure
plot(sim.t,sim.R);
ylabel('Range (m)')
xlabel('Elapsed time (sec)')
grid on

figure
hold on;
plot(sim.t, sim.av, 'LineWidth',2, 'Color', '#FF0000');
plot(sim.t, sim.tv, 'LineWidth',2, 'Color', '#0000FF');
plot(sim.t, sim.tvx, 'LineWidth',1, 'LineStyle',':', 'Color', '#0072BD');
plot(sim.t, sim.tvy, 'LineWidth',1, 'LineStyle','--', 'Color', '#0072BD');
plot(sim.t, sim.tvz, 'LineWidth',1, 'LineStyle','-.', 'Color', '#0072BD');
plot(sim.t, sim.avx, 'LineWidth',1, 'LineStyle',':', 'Color', '#D95319');
plot(sim.t, sim.avy, 'LineWidth',1, 'LineStyle','--', 'Color', '#D95319');
plot(sim.t, sim.avz, 'LineWidth',1, 'LineStyle','-.', 'Color', '#D95319');
legend('Attacker', 'Target', 'Target vx', 'Target vy',' Target vz',...
    'Attacker vx', 'Attacker vy',' Attacker vz',...
    'Location', 'eastoutside');
ylabel('velocity (m/s)')
xlabel('Elapsed time (s)')
hold off;


% Position
figure;
grid on
hold on;
% quiver3(sim.ax(:,1:end-1), sim.ay(:,1:end-1), sim.az(:,1:end-1), sim.avx(:,1:end-1), sim.avy(:,1:end-1), sim.avz(:,1:end-1));
% quiver3(sim.tx, sim.ty, sim.tz, sim.tvx, sim.tvy, sim.tvz);
% quiver3(sim.ax, sim.ay, sim.az, sim.ZEMn(:,1), sim.ZEMn(:,2), sim.ZEMn(:,3));
% quiver3(sim.ax, sim.ay, sim.az, sim.ZEM(:,1), sim.ZEM(:,2), sim.ZEM(:,3));
scatter3(sim.ax, sim.ay, sim.az, 10,'filled', 'DisplayName','Attacker');
scatter3(sim.tx, sim.ty, sim.tz, 10,'filled','DisplayName','Target');
for i = 1:200:size(sim.ax,2)
    plot3([sim.ax(:,i) sim.tx(:,i)],[sim.ay(:,i) sim.ty(:,i)],[sim.az(:,i) sim.tz(:,i)], Color='black');
end
title(['Proportional Navigation with ZEM, N = ' num2str(N) ])
legend('Attacker', 'Target', 'Location', 'southoutside', 'Orientation','horizontal')
xlabel('X')
ylabel('Y')
zlabel('Z')
view(45, 45)
hold off
