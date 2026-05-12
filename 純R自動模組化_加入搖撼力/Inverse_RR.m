function [phi, psi, phi_d, psi_d, phi_dd, psi_dd] = Inverse_RR(c,b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd,I)

%  Inverse Position Analysis

X = Xa - Xq; Y = Ya - Yq;

A = 2 * c * Y;
B = 2 * c * X;
C = c^2 + X^2 + Y^2 - b^2;

t1 = (A + sqrt(A^2 + B^2 - C^2)) / (C + B); t2 = (A - sqrt(A^2 + B^2 - C^2)) / (C + B);

if I == 1
    phi = 2*atan2((A + sqrt(A^2 + B^2 - C^2)), (C + B));
else
    phi=2*atan2((A - sqrt(A^2 + B^2 - C^2)),(C + B));
end 


U = (X- c*cos(phi))/b; V = (Y- c*sin(phi))/b;

psi = atan2(V,U);

%  Inverse Velocity Analysis

D = [ -c * sin(phi)  -b * sin(psi),
       c * cos(phi)   b * cos(psi)  ];
   
E = [ Xa_d-Xq_d;
      Ya_d-Yq_d];
  
F = inv(D)*E ; % F = D \ E ;

phi_d = F(1,1); psi_d = F(2,1);

%  Inverse Acceleration Analysis


G = [ -c * sin(phi)  -b * sin(psi),
       c * cos(phi)   b * cos(psi)  ];
   
H = [ Xa_dd-Xq_dd + c * cos(phi) * phi_d ^2 + b * cos(psi) * psi_d ^2  ,
      Ya_dd-Yq_dd + c * sin(phi) * phi_d ^2 + b * sin(psi) * psi_d ^2  ] ;
  
I = inv(G)*H; % I = G \ H;

phi_dd = I(1,1); psi_dd = I(2,1);

% phi = phi / pi * 180; psi = psi / pi * 180;

