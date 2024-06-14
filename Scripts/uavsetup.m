function UAVParams=uavsetup(pertubations)
%% set up UAV
    load('UAV_NominalParameters.mat');
    if pertubations==true
        %with perturbations
        UAVParams.IxxVal       = (1+(-0.1+(0.1--0.1)*rand))*IxxVal;
        UAVParams.IyyVal       = (1+(-0.1+(0.1--0.1)*rand))*IyyVal;
        UAVParams.IzzVal       = (1+(-0.1+(0.1--0.1)*rand))*IzzVal;
        UAVParams.kVal         = (1+(-0.1+(0.1--0.1)*rand))*kVal;
        UAVParams.lVal         = (1+(-0.1+(0.1--0.1)*rand))*lVal;
        UAVParams.mVal         = (1+(-0.1+(0.1--0.1)*rand))*mVal;
        UAVParams.bVal         = (1+(-0.1+(0.1--0.1)*rand))*bVal;
        UAVParams.omegaMax2Val = (1+(-0.1+(0.1--0.1)*rand))*omegaMax2Val;
    else
        %without perturbations
        UAVParams.IxxVal       = IxxVal;
        UAVParams.IyyVal       = IyyVal;
        UAVParams.IzzVal       = IzzVal;
        UAVParams.kVal         = kVal;
        UAVParams.lVal         = lVal;
        UAVParams.mVal         = mVal;
        UAVParams.bVal         = bVal;
        UAVParams.omegaMax2Val = omegaMax2Val;
    end
end