function cineq = myIneqConFunction(X, mindistance, attackTrajectory, ObjectDataLog, param, D,iter)
    %% making sure distance doesn't exceed initial distance
    p=param.p;
    AT = reshape(attackTrajectory,12,param.p+1);
    X1 = X(2:p+1,1);
    X2 = X(2:p+1,2);
    X3 = X(2:p+1,3);
    A1 = AT(1,2:p+1)';
    A2 = AT(2,2:p+1)';
    A3 = AT(3,2:p+1)';
    distanceconstraint = (X1-A1).^2 + (X2-A2).^2 + (X3-A3).^2 - mindistance^2;
    distanceconstraint2 = (X1-A1).^2 + (X2-A2).^2 - mindistance^2;

    %% maximum velocties for defenders
    V1 = X(2:p+1,7);
    V2 = X(2:p+1,8);
    V3 = X(2:p+1,9);
    velocityconstraint = (V1.^2 + V2.^2 + V3.^2).^0.5 - param.defender{D}.maxvelocity;
    

    %% obstacle constraints
    obstacleconstraint = [];
    
    %avoid objects
    for i=1:size(ObjectDataLog,2)
        O1 = ObjectDataLog{i}.x(1,iter:param.p+iter-1)';
        O2 = ObjectDataLog{i}.y(1,iter:param.p+iter-1)';
        O3 = ObjectDataLog{i}.z(1,iter:param.p+iter-1)';
        con = (param.object{i}.r+0.5)^2-((X1-O1).^2 + (X2-O2).^2 + (X3-O3).^2).^0.5;
        obstacleconstraint = [obstacleconstraint; con];
    end

    cineq = [distanceconstraint; velocityconstraint; obstacleconstraint];

end