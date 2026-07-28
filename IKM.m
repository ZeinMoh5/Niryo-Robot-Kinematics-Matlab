function [final_solutions] = IKM(Ex, Ey, Ez, Psi, Theta, Phi)

syms theta1 theta2 theta3 theta4 theta5 theta6
syms L1 L2 L3 L4 L5 L6 Tool_L

Ls = [L1, L2, L3, L4, L5, L6, Tool_L];

%-----------Denavit-Hartenberg Table-------------
DH = [
    %----------Arm---------
    0,      pi/2,   L1,      theta1;
    L2,     0,      0,       theta2 + pi/2;
    L3,     pi/2,   0,       theta3;
    0,      0,      L4,      0;
    %----------Wrist---------
    0,     -pi/2,   0,       theta4;
    0,      pi/2,   0,       theta5;
    0,      0,      0,       theta6;
    %----------End Effector----------
    0,      pi/2,   L5,      pi/2;
    0,      0,     -L6,      pi/2;
    Tool_L, 0,      0,       0
];

T01 = DenavitHartenberg(DH(1, 1), DH(1, 2), DH(1, 3), DH(1, 4));
T12 = DenavitHartenberg(DH(2, 1), DH(2, 2), DH(2, 3), DH(2, 4));
T23 = DenavitHartenberg(DH(3, 1), DH(3, 2), DH(3, 3), DH(3, 4));
T34 = DenavitHartenberg(DH(4, 1), DH(4, 2), DH(4, 3), DH(4, 4));
T45 = DenavitHartenberg(DH(5, 1), DH(5, 2), DH(5, 3), DH(5, 4));
T56 = DenavitHartenberg(DH(6, 1), DH(6, 2), DH(6, 3), DH(6, 4));
T67 = DenavitHartenberg(DH(7, 1), DH(7, 2), DH(7, 3), DH(7, 4));
T78 = DenavitHartenberg(DH(8, 1), DH(8, 2), DH(8, 3), DH(8, 4));
T89 = DenavitHartenberg(DH(9, 1), DH(9, 2), DH(9, 3), DH(9, 4));
T9E = DenavitHartenberg(DH(10, 1), DH(10, 2), DH(10, 3), DH(10, 4));

T0a = simplify(T01 * T12 * T23 * T34);
Ro = simplify(T45 * T56 * T67)
Twe = simplify(T78 * T89 * T9E);

Te = eye(4);
Te(1:3, 1:3) = Euler_Matrix(Psi, Theta, Phi);
Te(1:3, 4) = [Ex; Ey; Ez];

L1 = 183;
L2 = 210;
L3 = 30;
L4 = 221.5;
L5 = 23.7;
L6 = 5.5;
Tool_L = 0;

L = [L1, L2, L3, L4, L5, L6, Tool_L];
T0a = subs(T0a, Ls, L);
Twe = subs(Twe, Ls, L);

Tw=Te/double(Twe);

Px=Tw(1, 4);
Py=Tw(2, 4);
Pz=Tw(3, 4);

z = Pz - L1;
q1a = atan2(-Py, -Px);
q1b = wrapToPi(q1a + pi);

q1 = [q1a; q1b];
qa = zeros(4, 3);
nq = 0;

for i = 1:2
    if abs(cos(q1(i))) > abs(sin(q1(i)))
        r = -Px / cos(q1(i));
    else
        r = -Py / sin(q1(i));
    end

    D = (r^2 + z^2 - L2^2 - L3^2 - L4^2) / (2 * L2);
    h = sqrt(L3^2 + L4^2);
    c = D / h;
    p = atan2(L4, L3);
    q3 = [wrapToPi(p + acos(c));
          wrapToPi(p - acos(c))];

    for j = 1:2
        a = L2 + L3 * cos(q3(j)) + L4 * sin(q3(j));
        b = L3 * sin(q3(j)) - L4 * cos(q3(j));

        A = [a, b; -b, a];
        B = [r; z];

        C = A \ B;
        q2 = wrapToPi(atan2(C(1), C(2)));

        nq = nq + 1;
        qa(nq, :) = [q1(i), q2, q3(j)];
    end
end


sols = zeros(8, 6);
n = 0;

for i = 1:4
    th1 = qa(i, 1);
    th2 = qa(i, 2);
    th3 = qa(i, 3);

    Ta = subs(T0a, [theta1, theta2, theta3], sym([th1, th2, th3], 'd'));
    Tm = double(Ta) \ Tw;
    R = Tm(1:3, 1:3);

    q5 = [acos(R(3, 3)), -acos(R(3, 3))];

    for j = 1:2
        th5 = q5(j);
        s5 = sin(th5);

        if abs(s5) < 1e-9
            th4 = 0;
            th6 = atan2(R(2, 1), R(1, 1));
        else
            th4 = atan2(R(2, 3) / s5, R(1, 3) / s5);
            th6 = atan2(R(3, 2) / s5, -R(3, 1) / s5);
        end

        n = n + 1;
        sols(n, :) = wrapToPi([th1, th2, th3, th4, th5, th6]);
    end
end

sols = sols(1:n, :);

lim = [-175    175;
        -90   36.7;
        -80     90;
       -175    175;
       -100    110;
       -147.5 147.5] * pi / 180;

ok = true(size(sols, 1), 1);
for i = 1:6
    ok = ok & sols(:, i) >= lim(i, 1) & sols(:, i) <= lim(i, 2);
end

final_solutions = sols(ok, :);

end
