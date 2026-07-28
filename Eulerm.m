function [Psi, Theta, Phi] = Eulerm(Orientation)

r33 = max(min(Orientation(3, 3), 1), -1);
Theta = acos(r33);

if sin(Theta) == 0
    disp('EULER CANNOT BE IMPLEMMTED');
    Phi = 0;
    Psi = atan2(Orientation(2, 1), Orientation(1, 1));
else
    Psi = atan2(Orientation(1, 3), -Orientation(2, 3));
    Phi = atan2(Orientation(3, 1), Orientation(3, 2));
end

end
