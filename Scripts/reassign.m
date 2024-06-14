function [pairings, MinDistance]=reassign(param, DefenderDataLog, AttackerDataLog, MiscDataLog, iter)
    %get number of objects
    Attackerleft = setdiff(1:1:size(param.attacker,2), param.eliminated);
    noA = size(param.attacker,2);
    noD = size(param.defender,2);
    noAl = size(Attackerleft,2);
    
    if noAl == 0
        pairings = [];
        MinDistance = [];
    else
        %% set up cost matrix for assignment
        maxn = max(noA, noD);
        costMatrix = inf(maxn, maxn);

        %fill with euclidean distance
        for i=1:noD
            for k=1:noAl
                j = Attackerleft(k);
                costMatrix(i ,j) = norm([(DefenderDataLog{i}.state(1,iter-1)-AttackerDataLog{j}.x(iter-1)), ...
                    (DefenderDataLog{i}.state(2,iter-1)-AttackerDataLog{j}.y(iter-1)), ...
                    (DefenderDataLog{i}.state(3,iter-1)-AttackerDataLog{j}.z(iter-1))])+100*(MiscDataLog{j}.hitTime-((iter-1)*param.Ts));
            end
        end
    
        
        %% solve assignment
        [pairings, totalMinDistance] = matchpairs(costMatrix, 1000);
        totalMinDistance = sum(costMatrix(sub2ind(size(costMatrix), pairings(:, 1), pairings(:, 2))));


        for i=size(pairings,1):-1:1
            MinDistance(i)=norm([(DefenderDataLog{i}.state(1,iter-1)-AttackerDataLog{i}.x(iter-1)), ...
                    (DefenderDataLog{i}.state(2,iter-1)-AttackerDataLog{i}.y(iter-1)), ...
                    (DefenderDataLog{i}.state(3,iter-1)-AttackerDataLog{i}.z(iter-1))]);
        end
        
    end
end