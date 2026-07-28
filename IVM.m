function qdot = IVM(q, Xdot)

L1= 183;
L2= 210;
L3= 30;
L4= 221.5;
L5= 23.7;
L6= 5.5;
Tool_L= 0;

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);
t5 = q(5);
t6 = q(6);

%-----------Denavit-Hartenberg Table-------------
DH_T = [
    %----------Arm---------
    0,      pi/2,   L1,      t1;
    L2,     0,      0,       t2 + pi/2;
    L3,     pi/2,   0,       t3;
    0,      0,      L4,      0;
    %----------Wrist---------
    0,     -pi/2,   0,       t4;
    0,      pi/2,   0,       t5;
    0,      0,      0,       t6;
    %----------End Effector----------
    0,      pi/2,   L5,      pi/2;
    0,      0,     -L6,      pi/2;
    Tool_L, 0,      0,       0
];

T01=DenavitHartenberg(DH_T(1,1),DH_T(1,2),DH_T(1,3),DH_T(1,4));
T12=DenavitHartenberg(DH_T(2,1),DH_T(2,2),DH_T(2,3),DH_T(2,4));
T23=DenavitHartenberg(DH_T(3,1),DH_T(3,2),DH_T(3,3),DH_T(3,4));
T34=DenavitHartenberg(DH_T(4,1),DH_T(4,2),DH_T(4,3),DH_T(4,4));
T45=DenavitHartenberg(DH_T(5,1),DH_T(5,2),DH_T(5,3),DH_T(5,4));
T56=DenavitHartenberg(DH_T(6,1),DH_T(6,2),DH_T(6,3),DH_T(6,4));
T67=DenavitHartenberg(DH_T(7,1),DH_T(7,2),DH_T(7,3),DH_T(7,4));
T78=DenavitHartenberg(DH_T(8,1),DH_T(8,2),DH_T(8,3),DH_T(8,4));
T89=DenavitHartenberg(DH_T(9,1),DH_T(9,2),DH_T(9,3),DH_T(9,4));
T9E=DenavitHartenberg(DH_T(10,1),DH_T(10,2),DH_T(10,3),DH_T(10,4));

T02 = T01*T12;
T03_before_offset = T02*T23;
T03 = T03_before_offset*T34;
T04 = T03*T45;
T05 = T04*T56;
T06 = T05*T67;
T0E = T06*T78*T89*T9E;

z0 = [0;0;1]; 
origin0 = [0;0;0];
z1 = T01(1:3,3); 
origin1 = T01(1:3,4);
z2 = T02(1:3,3); 
origin2 = T02(1:3,4);
z3 = T03(1:3,3); 
origin3 = T03(1:3,4);
z4 = T04(1:3,3); 
origin4 = T04(1:3,4);
z5 = T05(1:3,3); 
origin5 = T05(1:3,4);
PosE = T0E(1:3,4);

J1 = [cross(z0, PosE - origin0); z0];
J2 = [cross(z1, PosE - origin1); z1];
J3 = [cross(z2, PosE - origin2); z2];
J4 = [cross(z3, PosE - origin3); z3];
J5 = [cross(z4, PosE - origin4); z4];
J6 = [cross(z5, PosE - origin5); z5];

J = [J1 J2 J3 J4 J5 J6];

qdot=pinv(J)*Xdot;

end
