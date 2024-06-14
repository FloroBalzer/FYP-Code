function plotSim(AttackerDataLog, TargetDataLog, DefenderDataLog, ObjectDataLog, MiscDataLog, param, iter, num);
if num ~= 0
    separate = 0;
    atj = 0; %plot attacker traj
    itj = 1; %plot intercept traj
    av = 0; %plot attacker vel
    dv = 1; %plot defender vel
    atd = 1; %plot attacker target dist
    dad = 1; %plot defender attacker dist
    ic = 0; %plot initi cond
    st = 1; %plot sim time
    tst = 0;
    th = 0; %plot thrusters
    

    total=0;
    total = atj + itj +av + dv+ atd+ dad+ ic+ st+tst;
    if separate==0
        figure
        tiledlayout(ceil(total/2),2);
    end 
   if atj == 1
    %% Plot Attacker Trajectories
    if separate == 1
        figure('Name','Attack Trajectories');
    else 
        nexttile
    end

    grid on; hold on;
    %plot attacker trajectories
    for i=1:size(AttackerDataLog,2)%saveas
        atplot=scatter3(AttackerDataLog{i}.x, AttackerDataLog{i}.y, AttackerDataLog{i}.z,1,[0.8500 0.3250 0.0980], ...
            'DisplayName', 'Attacker');
    end
    %plot defender 
    for i=1:size(param.defender,2)
        dtplot=scatter3(param.defender{i}.x0, param.defender{i}.y0, param.defender{i}.z0,20,[0 0.4470 0.7410], ...
            'Marker','s' ,'DisplayName','Defender');
    end
    %plot target trajectory
    ttplot=scatter3(TargetDataLog{param.maxtimeattacker}.x, TargetDataLog{param.maxtimeattacker}.y, TargetDataLog{param.maxtimeattacker}.z, ...
        5, [0.9290 0.6940 0.1250],'DisplayName', 'Target');
    %plot features
    title('Attack Trajectories');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    l = [atplot; dtplot; ttplot];
    legend(l,'attacker','defender','target','Location','eastoutside');
    view(45, 45);
    hold off
    
    if separate == 1
        saveas(gcf,['test',num2str(num), '_', num2str(1),' Attack Trajectories']);
    end
    
   end

   if itj == 1
    %% Plot Defender Trajectories with intercepts
    if separate == 1
        figure('Name','Intercept Trajectories');
    else 
        nexttile
    end
 
    grid on; hold on;

    %plot intercept trajectories
    for i=1:size(param.defender,2)
        dtplot=scatter3(DefenderDataLog{i}.state(1,:),DefenderDataLog{i}.state(2,:) , DefenderDataLog{i}.state(3,:),3,[0 0.4470 0.7410], ...
            'Marker','s' ,'DisplayName','Defender');
    end

    for i=1:size(AttackerDataLog,2)
        atplot=scatter3(AttackerDataLog{i}.x(1:AttackerDataLog{i}.interceptcyle), ...
            AttackerDataLog{i}.y(1:AttackerDataLog{i}.interceptcyle), ...
            AttackerDataLog{i}.z(1:AttackerDataLog{i}.interceptcyle),3,[0.8500 0.3250 0.0980], ...
            'DisplayName', 'Attacker');
    end

    %plot Obstacle trajectories
    otplot=[];
    for i=1:size(param.object,2)
        otplot = scatter3(ObjectDataLog{i}.x,ObjectDataLog{i}.y,ObjectDataLog{i}.z,10*param.object{i}.r, ...
            [0.4660 0.6740 0.1880],'DisplayName', 'Object');
    end
    %plot target trajectory
    ttplot=scatter3(TargetDataLog{param.maxtimeattacker}.x, TargetDataLog{param.maxtimeattacker}.y, TargetDataLog{param.maxtimeattacker}.z, ...
        5, [0.9290 0.6940 0.1250],'DisplayName', 'Target');
    %plot features
    title('Intercept Trajectories');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    % xlim([-30 30]); ylim([-30 30]); zlim([0 30])
    if isempty(otplot)
        l = [atplot; dtplot; ttplot];
        legend(l,'attacker','defender', 'target','Location','eastoutside');
    else
        l = [atplot; dtplot; otplot; ttplot];
        legend(l,'attacker','defender','obstacle', 'target','Location','eastoutside');
    end
    view(45, 45);
    hold off

    if separate ==1
        saveas(gcf,['test',num2str(num), '_', num2str(2),' Intercept Trajectories']);
    end
    
   end

   if tst==1
        if separate == 1
            figure('Name','Total Simultion Time per Defender');
        else 
            nexttile
        end
        hold on
        grid on;
        for i=1:size(param.defender,2)
            bar(i,param.DefenderRunTime{1,i}, 'Facecolor', [0 0.4470 0.7410])
        end
        hold off
        title('Total Simultion Time per Defender');
        xlabel('Defender'); ylabel('Time');xticks(1:1:size(param.defender,2));
    end

    if av == 1
    %% Plot Attacker Velocities
    if separate == 1
        figure('Name','Attacker Velocities');
    else 
        nexttile
    end
    
    hold on
    %plot attacker overall velocities
    for i=1:size(AttackerDataLog,2)
        plot(TargetDataLog{i}.t, AttackerDataLog{i}.v, 'DisplayName',['Attacker' num2str(i) ' velocity']);
    end
    %plot target velocity
    plot(TargetDataLog{param.maxtimeattacker}.t, TargetDataLog{param.maxtimeattacker}.v, 'DisplayName','Target velocity');
    %plot features
    title('Attacker Velocities');
    ylabel('velocity (m/s)'); xlabel('time (s)');
    legend('Location','eastoutside')
    %%plot hit time information 
    % for i=1:size(AttackerDataLog,2)
    %     T = TargetDataLog{i}.t(~isnan(TargetDataLog{i}.t));
    %     V = AttackerDataLog{i}.v(~isnan(AttackerDataLog{i}.v));
    %     datatip(scatter(T(end),V(end),'Marker','s','DisplayName', ['Impact time Attacker' num2str(i)],'HandleVisibility','off'),'FontSize',6);
    % end
    hold off
    if separate==1
        saveas(gcf,['test',num2str(num), '_', num2str(3),' Attacker Velocities']);
    end
    end

    if dv == 1
    %% Plot Defender Velocities
    if separate == 1
        figure('Name','Defender Velocities');
    else 
        nexttile
    end
    
    hold on
    %plot defender overall velocities
    for i=1:size(DefenderDataLog,2)
        plot(0:param.Ts:(size(DefenderDataLog{i}.v,2)-1)*param.Ts,DefenderDataLog{i}.v, 'DisplayName',['Defender' num2str(i)]);
    end
    %plot features
    title('Defender Velocities');
    ylabel('velocity (m/s)'); xlabel('time (s)');
    legend('Location','eastoutside')
    hold off

    if separate ==1
        saveas(gcf,['test',num2str(num), '_', num2str(4),' Defender Velocities']);
    end
    end

    if atd ==1
    %% Plot Attacker-Target Distance
    if separate == 1
        figure('Name','Attacker-Target Distance');
    else 
        nexttile
    end
    
    hold on
    %plot distance between attacker and target
    for i=1:size(AttackerDataLog,2)
        plot(TargetDataLog{i}.t, MiscDataLog{i}.R, 'DisplayName',['Attacker' num2str(i)]);
    end
    %plot features
    title('Attacker-Target Distance');
    ylabel('Distance (m)'); xlabel('Time (t)');
    legend('Location','eastoutside')
    hold off
    
    if separate == 1
        saveas(gcf,['test',num2str(num), '_', num2str(5),' Attacker-Target Distance']);
    end
    end
    
    if dad == 1
    %% Plot Defender-Attacker Distance
    if separate == 1
        figure('Name','Defender-Attacker Distance');
    else 
        nexttile
    end
    
    hold on
    %plot distance between attacker and target
    for i=1:size(DefenderDataLog,2)
        plot(0:param.Ts:(size(DefenderDataLog{i}.ADistance,2)-1)*param.Ts,DefenderDataLog{i}.ADistance, 'DisplayName',['Defender' num2str(i)]);
    end
    %plot features
    title('Defender-Attacker Distance');
    ylabel('Distance (m)'); xlabel('Time (t)');
    legend('Location','eastoutside')
    hold off
    if separate ==1
        saveas(gcf,['test',num2str(num), '_', num2str(6),' Defender-Attacker Distance']);
    end
    end

    if ic == 1
    %% Plot initial setup
    if separate == 1
        figure('Name','Initial conditions');
    else 
        nexttile
    end
    
    grid on; hold on;
    %plot attacker initial positions and velocities
    for i=1:size(AttackerDataLog,2)
        atplot=scatter3(param.attacker{i}.x0, param.attacker{i}.y0, param.attacker{i}.z0,30,[0.8500 0.3250 0.0980],'DisplayName', ['Attacker ', num2str(i)]);
        quiver3(param.attacker{i}.x0, param.attacker{i}.y0, param.attacker{i}.z0, param.attacker{i}.vx0, param.attacker{i}.vy0, param.attacker{i}.vz0,...
            'Color', '#D95319','DisplayName', ['Attacker' num2str(i)], 'HandleVisibility','off');
    end
    %plot defender initial positions, initial velocties are zero
    for i=1:size(param.defender,2)
        dtplot=scatter3(param.defender{i}.x0, param.defender{i}.y0, param.defender{i}.z0,30,[0 0.4470 0.7410],'filled','Marker','s' ,'DisplayName',['Defender ',num2str(i)]);
    end

    %plot defender initial positions, initial velocties are zero
    otplot=[];
    for i=1:size(param.object,2)
        otplot=scatter3(param.object{i}.x0, param.object{i}.y0, param.object{i}.z0,30,[0.4660 0.6740 0.1880],'filled','Marker','d' ,'DisplayName','Obstacle');
    end

    ttplot=scatter3(param.target.x0, param.target.y0, param.target.z0, 50, [0 0.4470 0.7410],'Marker','o' ,'DisplayName','Target');

    %plot assignment
    pairings = param.assignment.pairings;
    for i=1:size(pairings,1)
        d = pairings(i,1);
        a = pairings(i,2);
        plot3([param.defender{d}.x0 param.attacker{a}.x0],[param.defender{d}.y0 param.attacker{a}.y0],[param.defender{d}.z0 param.attacker{a}.z0] ...
            ,'--','Color','black', 'HandleVisibility','off');
    end

    %plot features
    title('Initial Conditions')
    xlabel('X'); ylabel('Y'); zlabel('Z');
    if isempty(otplot)
        l = [atplot; dtplot; ttplot];
        legend(l,'attacker','defender', 'target','Location','eastoutside');
    else
        l = [atplot; dtplot; otplot; ttplot];
        legend(l,'attacker','defender','obstacle', 'target','Location','eastoutside');
    end
    view(45, 45)
    hold off
    if separate ==1
        saveas(gcf,['test',num2str(num), '_', num2str(7),' Initial conditions']);
    end
    end
    
    if st ==1
     %% plot Simulation times
    if separate == 1
        figure('Name','Defender Simulation Time');
    else 
        nexttile
    end
    
    grid on; hold on;

    for i=1:size(param.defender,2)
        plot(1:size(param.timer{i},2),param.timer{i}, 'DisplayName',['Defender ',num2str(i)])
    end
    title('Defender Simulation Time');
    ylabel('Sim time (s)'); xlabel('Sample time (k)');
    legend('Location','eastoutside')
    hold off
    if separate == 1
     saveas(gcf,['test',num2str(num), '_', num2str(4),' Defender Simulation Time']);
    end
    end

    if separate == 0
        saveas(gcf,['test',num2str(num)]);
    end

    if th == 1
        %% plot thrusters per defender
    for i=1:size(DefenderDataLog,2)
        figure('Name',['Inputs Defender' num2str(i)]);
        tiledlayout(2,2);

        nexttile;
        stairs(0:param.Ts:(size(DefenderDataLog{i}.inputs,1)-1)*param.Ts, DefenderDataLog{i}.inputs(:,1));
        title('U1');
        xlabel('Time (t)'); ylabel('Thrust');

        nexttile;
        stairs(0:param.Ts:(size(DefenderDataLog{i}.inputs,1)-1)*param.Ts, DefenderDataLog{i}.inputs(:,2));
        title('U2');
        xlabel('Time (t)'); ylabel('Thrust');

        nexttile;
        stairs(0:param.Ts:(size(DefenderDataLog{i}.inputs,1)-1)*param.Ts, DefenderDataLog{i}.inputs(:,3));
        title('U3');
        xlabel('Time (t)'); ylabel('Thrust');

        nexttile;
        stairs(0:param.Ts:(size(DefenderDataLog{i}.inputs,1)-1)*param.Ts, DefenderDataLog{i}.inputs(:,4));
        title('U4');
        xlabel('Time (t)'); ylabel('Thrust');
        saveas(gcf,['test',num2str(num), '_','Inputs Defender',num2str(i)]);
    end
    end

    
v=0;
else

    %% Plot Defender Trajectories with intercepts for animation

    %plot intercept trajectories
    for j=1:iter+1
        clf;
        grid on; hold on;

        for i=1:size(param.defender,2)
            if size(DefenderDataLog{i}.state,2)>=j
                dtplot=scatter3(DefenderDataLog{i}.state(1,j),DefenderDataLog{i}.state(2,j) , ...
                    DefenderDataLog{i}.state(3,j),50,[0 0.4470 0.7410], ...
                    'filled','Marker','s' ,'DisplayName','Defender');
                dtplot2=scatter3(DefenderDataLog{i}.state(1,1:j),DefenderDataLog{i}.state(2,1:j) , ...
                    DefenderDataLog{i}.state(3,1:j),5,[0 0.4470 0.7410], ...
                    'Marker','s' ,'DisplayName','Defender');
            else
                dtplot=scatter3(DefenderDataLog{i}.state(1,j),DefenderDataLog{i}.state(2,j) , ...
                    DefenderDataLog{i}.state(3,j),50,[0 0.4470 0.7410], ...
                    'AlphaData', 0.5 ,'filled','Marker','s' ,'DisplayName','Defender');
                dtplot2=scatter3(DefenderDataLog{i}.state(1,1:j),DefenderDataLog{i}.state(2,1:j) , ...
                    DefenderDataLog{i}.state(3,1:j),5,[0 0.4470 0.7410], ...
                    'AlphaData', 0.5 ,'Marker','s' ,'DisplayName','Defender');

            end

        end
        for i=1:size(param.attacker,2)
            if AttackerDataLog{i}.interceptcyle >=j
                atplot=scatter3(AttackerDataLog{i}.x(j), ...
                    AttackerDataLog{i}.y(j), ...
                    AttackerDataLog{i}.z(j),50, [0.8500 0.3250 0.0980], ...
                    'filled','DisplayName', 'Attacker');
                atplot2=scatter3(AttackerDataLog{i}.x(1:j), ...
                    AttackerDataLog{i}.y(1:j), ...
                    AttackerDataLog{i}.z(1:j),5,[0.8500 0.3250 0.0980], ...
                    'DisplayName', 'Attacker');
            else
                atplot3=scatter3(AttackerDataLog{i}.x(1:AttackerDataLog{i}.interceptcyle), ...
                    AttackerDataLog{i}.y(1:AttackerDataLog{i}.interceptcyle), ...
                    AttackerDataLog{i}.z(1:AttackerDataLog{i}.interceptcyle),3,[0.8500 0.3250 0.0980], ...
                    'MarkerEdgeAlpha', 0.1,'DisplayName', 'Attacker');
            end
        end

        ttplot=scatter3(TargetDataLog{param.maxtimeattacker}.x, TargetDataLog{param.maxtimeattacker}.y, TargetDataLog{param.maxtimeattacker}.z, ...
         10, [0.9290 0.6940 0.1250],'Marker','o' ,'DisplayName', 'Target');


        %plot features
        title('Intercept Trajectories');
        xlabel('X'); ylabel('Y'); zlabel('Z');
        %xlim([-30 30]); ylim([-30 30]); zlim([0 30])
        v=45+j/2;
        view(45+j/2, 25);
        pause(0.1)

        hold off;

    end
    for i=1:30
    pause(0.1)
        view(v+2*i, 25);
    end
end
    
end
