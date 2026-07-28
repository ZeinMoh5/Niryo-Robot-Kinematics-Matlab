function [ Orientation ] = Euler_Matrix( Psi,Theta,Phi )

Orientation(1,1)=cos(Psi)*cos(Phi)-sin(Psi)*cos(Theta)*sin(Phi);
Orientation(1,2)=-cos(Psi)*sin(Phi)-sin(Psi)*cos(Theta)*cos(Phi);
Orientation(1,3)=sin(Psi)*sin(Theta);

Orientation(2,1)=sin(Psi)*cos(Phi)+cos(Psi)*cos(Theta)*sin(Phi);
Orientation(2,2)=-sin(Psi)*sin(Phi)+cos(Psi)*cos(Theta)*cos(Phi);
Orientation(2,3)=-cos(Psi)*sin(Theta);

Orientation(3,1)=sin(Theta)*sin(Phi);
Orientation(3,2)=sin(Theta)*cos(Phi);
Orientation(3,3)=cos(Theta);

end

