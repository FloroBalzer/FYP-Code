function param =controllersetup(param, attackTrajectory, ObjectDataLog)

for i=size(param.assignment.pairings,1):-1:1
    D = param.assignment.pairings(i,1);
    defender = param.defender{D};

    A = param.assignment.pairings(i,2);
    attacker=param.attacker{A};

    aT=reshape(attackTrajectory{A}(:,1:param.p+1),[],1);
    mindistance = param.assignment.mindistance(i);
    
    
    %% Controller Set-Up
    controller = nlmpc(param.nx, param.ny, param.nu);
    controller.Ts = param.Ts;
    controller.PredictionHorizon = param.p;
    controller.ControlHorizon = param.c;
    controller.Model.NumberOfParameters = 1;
    controller.MV = struct(Min={0;0;0;0}, Max={1;1;1;1});
    
    controller.Model.StateFcn = @QuadrotorStateFcnBase;
    controller.Jacobian.StateFcn = @QuadrotorStateJacobianFcnBase;
    controller.Optimization.CustomCostFcn = @(X,U,e,data, N) myCostFunction(X, param.H, aT);
    controller.Optimization.CustomIneqConFcn = @(X,U,e,data,N) myIneqConFunction(X, mindistance, aT, ObjectDataLog,param, D,1);
    controller.Optimization.ReplaceStandardCost = true;
    % controller.Optimization.SolverOptions.Algorithm = 'sqp';
    % controller.Optimization.SolverOptions.Display = 'off';

    controller.Optimization.SolverOptions.MaxIter=param.maxiter;
    validateFcns(controller, rand(param.nx, 1), rand(param.nu, 1), [], {0})
    
    %% Options set up
    %calculate initial guess in direction of attacker
    ix = attacker.x0-defender.x0;
    iy = attacker.y0-defender.y0;
    iz = attacker.z0-defender.z0;
    iu = norm([ix iy iz]);
    x = (ix/iu) + defender.x0;
    y = (iy/iu) + defender.y0;
    z = (iz/iu) + defender.z0;
    
    options = nlmpcmoveopt;
    options.Parameters = {0}; %N
    options.X0 = [x y z 0 0 0 (ix/iu) (iy/iu) (iz/iu) 0 0 0];

    
    %% Save info
    param.controller{D}=controller;
    param.controlOptions{D} = options;
end


end