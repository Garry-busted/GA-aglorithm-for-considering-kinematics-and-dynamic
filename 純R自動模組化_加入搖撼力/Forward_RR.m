function [Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd] = Forward_RR(c, b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,phi, psi,phi_d, psi_d, phi_dd, psi_dd)

% phi = phi /180 * pi;
% psi = psi /180 * pi;

%  Forward Position Analysis

Xa=c*cos(phi)+b*cos(psi)+Xq;
Ya=c*sin(phi)+b*sin(psi)+Yq;

%  Forward Velocity Analysis

Xa_d=-c*phi_d*sin(phi)-b*psi_d*sin(psi)+Xq_d;
Ya_d=c*phi_d*cos(phi)+b*psi_d*cos(psi)+Yq_d;

%  Forward Acceleration Analysis  

Xa_dd=-c*phi_dd*sin(phi)-b*psi_dd*sin(psi)-c*cos(phi)*phi_d^2-b*cos(psi)*psi_d^2+Xq_dd;
Ya_dd=c*phi_dd*cos(phi)+b*psi_dd*cos(psi)-c*sin(phi)*phi_d^2-b*sin(psi)*psi_d^2+Yq_dd;
% 
% phi = phi / pi * 180;
% psi = psi / pi * 180;
