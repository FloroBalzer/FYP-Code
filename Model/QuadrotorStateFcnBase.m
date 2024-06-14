function out1 = QuadrotorStateFcnBase(in1,in2, N)
%QuadrotorStateFcnBase Function is sourced from Imperial College London, 
%Department of Electrical Engineering, Predictive Control Module ELEC70028

%OUT1 = QuadrotorStateFcnBase(IN1,IN2)


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
u = in1(7,:);
v = in1(8,:);
w = in1(9,:);
t2 = cos(phi);
t3 = cos(psi);
t4 = cos(theta);
t5 = sin(phi);
t6 = sin(psi);
t7 = sin(theta);
t10 = input_1+input_2+input_3+input_4;
t8 = r.*t2;
t9 = q.*t5;
t11 = t8+t9;
et1 = input_1.*2.471255962943901e+2-input_2.*2.471255962943901e+2-input_3.*2.471255962943901e+2+input_4.*2.471255962943901e+2;
et2 = q.*r.*(-4.856750729113056e-1);
et3 = input_1.*2.00933896052448e+2+input_2.*2.00933896052448e+2-input_3.*2.00933896052448e+2-input_4.*2.00933896052448e+2;
et4 = p.*r.*5.818105733017157e-1;
et5 = input_1.*(-6.659016492)+input_2.*6.659016492-input_3.*6.659016492;
et6 = input_4.*6.659016492-p.*q.*1.34e-1;
mt1 = [u;v;w;p+t11.*tan(theta);q.*t2-r.*t5;t11./t4;t10.*(t5.*t6+t2.*t3.*t7).*(-5.184807619047619);t10.*(t3.*t5-t2.*t6.*t7).*5.184807619047619;t2.*t4.*t10.*(-5.184807619047619)+9.81e+2./1.0e+2;et1+et2;et3+et4];
mt2 = [et5+et6];
out1 = [mt1;mt2];
end
