function [pairings, unassigned, MinDistance]= assign(param, MiscDataLog)
%get number of objects
noA = size(param.attacker,2);
noD = size(param.defender,2);

%% set up cost matrix for assignment
maxn = max(noA, noD);
costMatrix = inf(maxn, maxn);

%fill with euclidean distance
for i=1:noD
    for j=1:noA
        costMatrix(i ,j) = norm([(param.defender{i}.x0-param.attacker{j}.x0),(param.defender{i}.y0-param.attacker{j}.y0),(param.defender{i}.z0-param.attacker{j}.z0)])+100*MiscDataLog{j}.hitTime;
    end
end
%% solve assignment
[pairings, totalMinDistance] = matchpairs(costMatrix, 1000);
totalMinDistance = sum(costMatrix(sub2ind(size(costMatrix), pairings(:, 1), pairings(:, 2))));
for i=size(pairings,1):-1:1
    D = pairings(i,1);
    A = pairings(i,2);
    MinDistance(i)=norm([(param.defender{D}.x0-param.attacker{A}.x0),(param.defender{D}.y0-param.attacker{A}.y0),(param.defender{D}.z0-param.attacker{A}.z0)]);
end

unassigned = setdiff(1:1:noA, pairings(:,2));

end
