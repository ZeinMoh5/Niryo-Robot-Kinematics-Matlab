function [Ex, Ey, Ez, Alpha, Beta, Gamma, Psi, Theta, Phi] = DKM(t1, t2, t3, t4, t5, t6)

L1 = 183;
L2 = 210;
L3 = 30;
L4 = 221.5;
L5 = 23.7;
L6 = 5.5;
Tool_L = 0;

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

T0E = eye(4);
for i = 1:size(DH_T, 1)
    T0E = T0E * DenavitHartenberg(DH_T(i, 1), DH_T(i, 2), DH_T(i, 3), DH_T(i, 4));
end

Ex = T0E(1, 4);
Ey = T0E(2, 4);
Ez = T0E(3, 4);

Orientation = T0E(1:3, 1:3);

[Alpha, Beta, Gamma] = RPY(Orientation);
[Psi, Theta, Phi] = Eulerm(Orientation);

end
