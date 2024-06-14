function param = initialise(mindefenders, maxdefenders, minattackers, maxattackers, ...
    statobjetcs, dynobjetcs ,targetvx, targetvy, targetvz)
%% Variables 
param.Tt=200; %simulation time
param.Ts = 0.05; %sampling time

%sim variables
param.hitDistance = 0.2; %tolerance for accepted hit count
param.maxhitTime=0; %initialise shortest hit time
param.minhitTime=param.Tt;
param.maxtimeattacker=1;
param.TargetSurvive = true;
param.perturbations = true;

%controller related variables
param.nx = 12; %input state dimensions
param.ny = 12; %output state dimensions
param.nu = 4; %control dimensions
param.p = 10; %prediction horizon
param.c = 3; %control horizon
param.maxiter = 10000;
param.ueq = 0.473; %equilibrium thrust
param.prevu = [0 0 0 0];

%% initial condition for target
param.target.x0 = 0;
param.target.y0 = 0;
param.target.z0 = 0;
param.target.vx0 = targetvx;
param.target.vy0 = targetvy;
param.target.vz0 = targetvz;

%% initial conditions for attackers
%upper and lower bounds for xy
param.init.arlow = 20;
param.init.arhigh = 30;

arlow=param.init.arlow;
arhigh=param.init.arhigh;

%upper and lower bounds for z
param.init.azlow = 10;
param.init.azhigh = 25;
azlow = param.init.azlow;
azhigh = param.init.azhigh;

%upper and lower bounds for velocities
param.init.avlow = 2.5;
param.init.avhigh = 3.5;

avlow = param.init.avlow;
avhigh = param.init.avhigh;


%% initial conditions for defenders
%upper and lower bounds for xy
param.init.drlow = 15;
param.init.drhigh = 20;

drlow=param.init.drlow;
drhigh=param.init.drhigh;

%upper and lower bounds for z
param.init.dzlow = 10;
param.init.dzhigh = 25;
dzlow = param.init.dzlow;
dzhigh = param.init.dzhigh;



%% generate random number of defenders and attackers within given range and assign initial val
numberofattackers = randi([minattackers maxattackers]);
rng("shuffle");
numberofdefenders = randi([mindefenders maxdefenders]);

for i=numberofattackers:-1:1
    rng("shuffle");
    theta = rand*2*pi;
    attacker.x0 = cos(theta)*(arlow+(arhigh-arlow)*rand);
    attacker.y0 = sin(theta)*(arlow+(arhigh-arlow)*rand);
    attacker.z0 = azlow+(azhigh-azlow)*rand;
    attacker.vx0 = (randi([0, 1]) * 2 - 1)*(avlow+(avhigh-avlow)*rand);
    attacker.vy0 = (randi([0, 1]) * 2 - 1)*(avlow+(avhigh-avlow)*rand);
    attacker.vz0 = (randi([0, 1]) * 2 - 1)*(avlow+(avhigh-avlow)*rand);
    param.attacker{i}=attacker;
end

for i=numberofdefenders:-1:1
    rng("shuffle");
    theta = rand*2*pi;
    defender.x0 = cos(theta)*(drlow+(drhigh-drlow)*rand);
    defender.y0 = sin(theta)*(drlow+(drhigh-drlow)*rand);
    defender.z0 = dzlow+(dzhigh-dzlow)*rand;
    param.defender{i}=defender;
    param.defender{i}.maxvelocity =15;
end

%initialise intercept time
param.intercepttime = [(1:1:numberofdefenders)',zeros(numberofdefenders,1)];


%% generate static and dynamic spherical objects
%initial conditions for objects
param.object = [];
%upper and lower bounds for object size
param.init.oslow = 1.6;
param.init.oshigh = 2;

oslow=param.init.oslow;
oshigh=param.init.oshigh;

%upper and lower bounds for xy
param.init.orlow = 0;
param.init.orhigh = 25;

orlow=param.init.orlow;
orhigh=param.init.orhigh;

%upper and lower bounds for z
param.init.ozlow = 0;
param.init.ozhigh = 25;
ozlow = param.init.ozlow;
ozhigh = param.init.ozhigh;

%upper and lower bounds for velocities
param.init.ovlow = 0.05;
param.init.ovhigh = 0.1;

ovlow = param.init.ovlow;
ovhigh = param.init.ovhigh;

%generate objects
for i=statobjetcs:-1:1
    rng("shuffle");
    theta = rand*2*pi;
    object.x0 = cos(theta)*(orlow+(orhigh-orlow)*rand);
    object.y0 = sin(theta)*(orlow+(orhigh-orlow)*rand);
    object.z0 = ozlow+(ozhigh-ozlow)*rand;
    object.r = oslow+(oshigh-oslow)*rand;
    object.static = 1;
    object.vx0 = 0;
    object.vy0 = 0;
    object.vz0 = 0;    
    param.object{i}=object;
end


for i = dynobjetcs:-1:1
    rng("shuffle");
    theta = rand*2*pi;
    object.x0 = cos(theta)*(orlow+(orhigh-orlow)*rand);
    object.y0 = sin(theta)*(orlow+(orhigh-orlow)*rand);
    object.z0 = ozlow+(ozhigh-ozlow)*rand;
    object.r = oslow+(oshigh-oslow)*rand;
    object.static = 0;
    object.vx0 = (randi([0, 1]) * 2 - 1)*(ovlow+(ovhigh-ovlow)*rand);
    object.vy0 = (randi([0, 1]) * 2 - 1)*(ovlow+(ovhigh-ovlow)*rand);
    object.vz0 = (randi([0, 1]) * 2 - 1)*(ovlow+(ovhigh-ovlow)*rand);
end

%% Cost matrix
Q=sparse(zeros(param.nx,param.nx));
Q(1,1)=15;
Q(2,2)=15;
Q(3,3)=15;
Q(4,4)=0;
Q(5,5)=0;
Q(6,6)=0;
Q(7,7)=1;
Q(8,8)=1;
Q(9,9)=1;
Q(10,10)=0;
Q(11,11)=0;
Q(12,12)=0;

H = sparse(zeros((param.p+1)*param.nx));
H(1:(param.p+1)*param.nx, 1:(param.p+1)*param.nx) = kron(eye(param.p+1), Q);
param.H =H;

%% overwrite values for testing here
% for i=1:7
%     param.attacker{i}.x0 = 20;
%     param.attacker{i}.y0 = 20;
%     param.attacker{i}.z0 = 20;
%     param.attacker{i}.vx0 = -2;
%     param.attacker{i}.vy0 = -2;
%     param.attacker{i}.vz0 = -2;
%     param.defender{i}.x0 = -20;
%     param.defender{i}.y0 = -20;
%     param.defender{i}.z0 = 27.5-2.5*i;
%     param.defender{i}.maxvelocity = 15;
% end

% param.attacker{1}.x0 = 20;
% param.attacker{1}.y0 = 20;
% param.attacker{1}.z0 = 20;
% param.attacker{1}.vx0 = -2;
% param.attacker{1}.vy0 = -2;
% param.attacker{1}.vz0 = -2;
% param.defender{1}.x0 = -20;
% param.defender{1}.y0 = -20;
% param.defender{1}.z0 = 20;
% param.defender{1}.maxvelocity = 15;
% 
% param.attacker{2}.x0 = 22;
% param.attacker{2}.y0 = 22;
% param.attacker{2}.z0 = 22;
% param.attacker{2}.vx0 = -2;
% param.attacker{2}.vy0 = -2;
% param.attacker{2}.vz0 = -2;



end
