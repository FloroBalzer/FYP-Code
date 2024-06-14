function [A,B] = QuadrotorStateJacobianFcnBase(in1,in2, N)
%QuadrotorStateJacobianFcnBase Function is sourced from Imperial College London, 
%Department of Electrical Engineering, Predictive Control Module ELEC70028

%[A,B] = QuadrotorStateJacobianFcnBase(IN1,IN2)


input_1 = in2(1,:);
input_2 = in2(2,:);
input_3 = in2(3,:);
input_4 = in2(4,:);
p = in1(10,:);
phi = in1(4,:);
psi = in1(6,:);
q = in1(11,:);
r = in1(12,:);
theta = in1(5,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t8 = tan(theta);
t13 = input_1+input_2+input_3+input_4;
t9 = q.*t2;
t10 = r.*t2;
t11 = q.*t5;
t12 = r.*t5;
t14 = 1.0./t4;
t18 = t2.*t4.*5.184807619047619;
t19 = t3.*t5.*5.184807619047619;
t20 = t5.*t6.*5.184807619047619;
t23 = t2.*t3.*t7.*5.184807619047619;
t24 = t2.*t6.*t7.*5.184807619047619;
t15 = -t12;
t16 = t10+t11;
t21 = -t18;
t22 = -t20;
t25 = -t23;
t26 = -t24;
t17 = t9+t15;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8.*t17,-t16,t14.*t17,t13.*(t2.*t6-t3.*t5.*t7).*(-5.184807619047619),t13.*(t2.*t3+t5.*t6.*t7).*5.184807619047619];
mt2 = [t4.*t5.*t13.*5.184807619047619,0.0,0.0,0.0,0.0,0.0,0.0,t16.*(t8.^2+1.0),0.0,t7.*t14.^2.*t16,t2.*t3.*t4.*t13.*(-5.184807619047619),t2.*t4.*t6.*t13.*(-5.184807619047619)];
mt3 = [t2.*t7.*t13.*5.184807619047619,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t13.*(t3.*t5-t2.*t6.*t7).*(-5.184807619047619),t13.*(t5.*t6+t2.*t3.*t7).*(-5.184807619047619),0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0];
mt4 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,r.*5.818105733017157e-1,q.*(-1.34e-1),0.0,0.0,0.0,t5.*t8,t2,t5.*t14,0.0,0.0,0.0,r.*(-4.856750729113056e-1),0.0,p.*(-1.34e-1),0.0,0.0,0.0,t2.*t8];
mt5 = [-t5,t2.*t14,0.0,0.0,0.0,q.*(-4.856750729113056e-1),p.*5.818105733017157e-1,0.0];
A = reshape([mt1,mt2,mt3,mt4,mt5],12,12);
if nargout > 1
    t27 = t19+t26;
    t28 = t22+t25;
    mt6 = [0.0,0.0,0.0,0.0,0.0,0.0,t28,t27,t21,2.471255962943901e+2,2.00933896052448e+2,-6.659016492,0.0,0.0,0.0,0.0,0.0,0.0,t28,t27,t21,-2.471255962943901e+2];
    mt7 = [2.00933896052448e+2,6.659016492,0.0,0.0,0.0,0.0,0.0,0.0,t28,t27,t21,-2.471255962943901e+2,-2.00933896052448e+2];
    mt8 = [-6.659016492,0.0,0.0,0.0,0.0,0.0,0.0,t28,t27,t21,2.471255962943901e+2,-2.00933896052448e+2,6.659016492];
    B = reshape([mt6,mt7,mt8],12,4);
end
end
