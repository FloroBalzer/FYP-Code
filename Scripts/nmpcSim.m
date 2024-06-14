function [y, x, param, DefenderDataLog] = nmpcSim(param, UAVParams, A, D, attackTrajectory, mindistance,DefenderDataLog, ObjectDataLog, iter,y,x,t)
    odeOpts = odeset( 'RelTol', 1e-3, 'MaxStep', 0.001 ); % ODE Options

    %% set up sim parameters
    [u,opt]= nlmpcmove(param.controller{D}, y,param.prevu, [], [], param.controlOptions{D});
    param.controller{D}.Optimization.CustomCostFcn = @(X,U,e,data, N) myCostFunction(X, param.H, reshape(attackTrajectory{A}(:,1+iter:param.p+1+iter),[],1));
    param.controller{D}.Optimization.CustomIneqConFcn = @(X,U,e,data,N) myIneqConFunction(X, mindistance, reshape(attackTrajectory{A}(:,1+iter:param.p+1+iter),[],1), ...
        ObjectDataLog,param, D,iter);
    %param.controlOptions{D}=opt;
    %Saturate the inputs to [0, 1] for the simulation
    usat = min( max( u, 0 ), 1 );

    %Setup the simulation model & Simulate
    mod = @(t, state) UAV_nl_model(usat, state, UAVParams{D});
    [tt, x] = ode23t( mod, t:0.001:t + param.Ts, x(end,:), odeOpts);

    %The output is simply all the states
    y = x(end,1:12)';

    %Variables for analysis
    DefenderDataLog{D}.state  = [DefenderDataLog{D}.state,  y];
    DefenderDataLog{D}.inputs  = [DefenderDataLog{D}.inputs;  u'];

    v = norm(y(7:9));
    DTDistance = norm(y(1:3)-attackTrajectory{A}(1:3,iter));
    DefenderDataLog{D}.ADistance = [DefenderDataLog{D}.ADistance,DTDistance];
    DefenderDataLog{D}.v=[ DefenderDataLog{D}.v, v];

    if DTDistance<param.hitDistance
        param.defender{D}.hittimer=param.defender{D}.hittimer+1;
    else
        param.defender{D}.hittimer=0;
    end
end