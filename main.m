%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% This file will run a complete simulation of a nonlinear UAV model
% with the designed controller in the file `myMPController.m`.
%
% Author: Floro Balzer
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
clear variables
close all
currentFolder=pwd;
addpath(strcat(currentFolder,'/Model'), strcat(currentFolder,'/scripts'), strcat(currentFolder,'/bin'),strcat(currentFolder,'/TestResults'))



setupstart=tic;
%% Initialise
%inputs for initialise: mindefenders, maxdefenders, minattackers, maxattackers, statobjetcs, dynobjetcs, 
% targetvx, targetvy, targetvz
param = initialise(1,1, 1, 1, 0, 0, 1, 1, 0);
%UAV parameters, with perturbation on/off
for i=size(param.defender,2):-1:1
    UAVParams{i}=uavsetup(param.perturbations);
end

%% Set up attacker trajectories
for i=size(param.attacker,2):-1:1
    [AttackerDataLog{i}, TargetDataLog{i}, MiscDataLog{i}] = pureNav(param, param.attacker{i}, 1);
    
    %set up state evolution of attacker
    z_v                 = zeros(1,sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerX           = AttackerDataLog{i}.x(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerY           = AttackerDataLog{i}.y(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerZ           = AttackerDataLog{i}.z(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerVX          = AttackerDataLog{i}.vx(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerVY          = AttackerDataLog{i}.vy(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackerVZ          = AttackerDataLog{i}.vz(1:sum(~isnan(AttackerDataLog{i}.x(:))));
    attackTrajectory{i} = [attackerX; attackerY; attackerZ; z_v; z_v; z_v; attackerVX; attackerVY; attackerVZ; z_v; z_v; z_v];
    hitState = [MiscDataLog{i}.hitPosition'; 0; 0; 0; MiscDataLog{i}.hitVelocity'; 0; 0; 0];
    attackTrajectory{i} = [attackTrajectory{i}, kron(ones(1,param.p), hitState)];

    %fprintf('attacker %i hit time : %.2f',i, MiscDataLog{i}.hitTime);
    %fprintf('hit distance attacker 1: %.2f\n', MiscDataLog1.hitDistance);
    %fprintf(' hit position: %2f\n', MiscDataLog{i}.hitPosition);

    if MiscDataLog{i}.hitTime > param.maxhitTime
        param.maxhitTime = MiscDataLog{i}.hitTime;
        param.maxtimeattacker=i;
    end
    if MiscDataLog{i}.hitTime < param.minhitTime
        param.minhitTime = MiscDataLog{i}.hitTime;
        param.mintimeattacker=i;
    end

    AttackerDataLog{i}.interceptcyle = MiscDataLog{i}.hitTime/param.Ts;
    AttackerDataLog{i}.interceptPos = MiscDataLog{i}.hitPosition;
    param.attacker{i}.flag=0;
    param.eliminated = [];
end

%% set up object trajectories
ObjectDataLog = [];
for i=size(param.object,2):-1:1
    if param.object{i}.static == 1
        ObjectDataLog{i}.x = kron(ones(1,param.Tt/param.Ts), param.object{i}.x0);
        ObjectDataLog{i}.y = kron(ones(1,param.Tt/param.Ts), param.object{i}.y0);
        ObjectDataLog{i}.z = kron(ones(1,param.Tt/param.Ts), param.object{i}.z0);
    else
        ObjectDataLog{i}.x(1) = param.object{i}.x0; 
        ObjectDataLog{i}.y(1) = param.object{i}.y0;
        ObjectDataLog{i}.z(1) = param.object{i}.z0;
        for k=2:param.Tt/param.Ts
            ObjectDataLog{i}.x(k) = ObjectDataLog{i}.x(k-1)+param.object{i}.vx0*param.Ts;
            ObjectDataLog{i}.y(k) = ObjectDataLog{i}.y(k-1)+param.object{i}.vy0*param.Ts;
            ObjectDataLog{i}.z(k) = ObjectDataLog{i}.z(k-1)+param.object{i}.vz0*param.Ts;
        end
    end
end

%% Assign Defenders to Targets

[pairings, unassigned, MinDistance]= assign(param, MiscDataLog);
param.assignment.mindistance = MinDistance;
param.assignment.pairings = pairings;
param.assignment.unassigned = unassigned;

%% Set up NMPC Controllers
param = controllersetup(param, attackTrajectory, ObjectDataLog);


 %% Run NMPC controller
%initialise
for i=size(param.defender,2):-1:1
    x{i} = [param.defender{i}.x0, param.defender{i}.y0, param.defender{i}.z0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    y{i} = x{i}(1:12)';
    DefenderDataLog{i}.inputs = [];
    DefenderDataLog{i}.state=y{i};
    DefenderDataLog{i}.ADistance=[];
    DefenderDataLog{i}.v=[];
    param.defender{i}.hittimer=0;
    param.timer{i}=[];
    param.timer2{i}=[];
end
setupend=toc(setupstart);

iter=1;
simstart=tic;
for t=0:param.Ts:param.maxhitTime
    for i=size(pairings,1):-1:1
        tm=0;
        tic;
        D = pairings(i,1);
        A = pairings(i,2);
        mindistance = MinDistance(i);
        [y{D},x{D}, param, DefenderDataLog] = nmpcSim(param, UAVParams, A, D, attackTrajectory, mindistance, ...
            DefenderDataLog, ObjectDataLog,iter,y{D}, x{D},t);
        DefenderDataLog{D}.interceptPos = y;
        [DefenderDataLog] = maintain(param, pairings,DefenderDataLog);
        tm=toc;
        param.timer{D} = [param.timer{D},tm];
    end
    % if iter>=2
    % [pairings, MinDistance]=reassign(param, DefenderDataLog, AttackerDataLog, MiscDataLog, iter);
    % end
    for i=size(pairings,1):-1:1
        tm=0;
        tic
        D = pairings(i,1);
        A = pairings(i,2);
        %monitor progress
        fprintf('D%d: %d\n',D,param.defender{D}.hittimer);
        if param.defender{D}.hittimer >= 1
            param.attacker{A}.flag=1;
            fprintf('Deleting pair: %d %d\n',D, A);
            param.eliminated = [param.eliminated, A];
            AttackerDataLog{A}.interceptcyle = iter;
            AttackerDataLog{A}.interceptPos = y;
            % pairings(i,:)=[];
            % MinDistance(i)=[];
            [pairings, MinDistance]=reassign(param, DefenderDataLog, AttackerDataLog, MiscDataLog, iter);
            param.defender{D}.hittimer=0;
        end
        % if norm(y{D}(1:3))>100
        %     [pairings, MinDistance]=reassign(param, DefenderDataLog, AttackerDataLog, MiscDataLog, iter);
        % end
        if t*param.Ts >= AttackerDataLog{A}.interceptcyle
            param.TargetSurvive = false;
            break;
        end
        tm=toc;
        param.timer2{D} = [param.timer2{D},tm];
    end
    

    if isempty(pairings)
        break;
    end

    if param.TargetSurvive ==false
        break;
    end
    
    iter=iter+1;
    fprintf('Cycle: %d, Time: %.2f\n',iter,t);
    fprintf('Eliminated: %d\n',size(param.eliminated,2));
end 
    
simend=toc(simstart);

%% Check Target Survival
for i=1:size(param.attacker,2)
    if param.attacker{i}.flag==0
        param.TargetSurvive = false;
    end
end

if size(param.eliminated)==size(param.attacker) & param.TargetSurvive == true
    disp('Target Survived');
else
    disp('Target Did Not Survive');
end

%% Plot Data
%fill empty DataLogs for plotting
for i=1:size(DefenderDataLog,2)
    if isempty(DefenderDataLog{i})
        DefenderDataLog{i}.inputs=0;
        DefenderDataLog{i}.state=0;
        DefenderDataLog{i}.ADistance=0;
        DefenderDataLog{i}.v=0;
        DefenderDataLog{i}.interceptPos=0;
    end
end


for i=1:size(param.defender,2)           
    param.DefenderRunTime{i} = sum(param.timer{i})+sum(param.timer2{i});
    % fprintf('Defender %d simtime: %.2f\n',i, DefenderRunTime{i});
end

fprintf('Set Up time: %.4f\n',setupend);
fprintf('Sim time: %4.f\n',simend);

plotSim(AttackerDataLog, TargetDataLog, DefenderDataLog, ObjectDataLog,  MiscDataLog, param, iter,0);

