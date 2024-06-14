function [DefenderDataLog] = maintain(param, pairings,DefenderDataLog)
    availabledefender = setdiff(1:1:size(param.defender,2), pairings(:,1)');
    for i=1:size(availabledefender,2)
        DefenderDataLog{availabledefender(i)}.state = [DefenderDataLog{availabledefender(i)}.state, DefenderDataLog{availabledefender(i)}.state(:,end)];
    end

end