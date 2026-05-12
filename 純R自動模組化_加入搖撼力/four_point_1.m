

Xo=0;
Yo=0;
X1=20;
Y1=10;
X3=40;
Y3=10;
r1=((X1-Xo)^2+(Y1-Yo)^2)^(1/2);
r2=35;
r3=30;
phi=atan2(Y1,X1);

for i=1:360
    [Xa(i),Ya(i),Xa_d(i),Ya_d(i),Xa_dd(i),Ya_dd(i)] = Forward_RR(r1,0, Xo, Yo,0,0,0,0,phi+(i-1)*pi/180,0,5,0, 0,0);
    [t4(i),t3(i), t4_d(i), t3_d(i), t4_dd(i), t3_dd(i)] = Inverse_RR(r3,r2, X3, Y3,0,0,0,0,Xa(i),Ya(i),Xa_d(i),Ya_d(i),Xa_dd(i),Ya_dd(i),2);
    [fx(i),fy(i),fx_d(i),fy_d(i),fx_dd(i),fy_dd(i)] = Forward_RR(r3,0, X3, Y3,0,0,0,0,t4(i),0,t4_d(i),0, t4_dd(i),0);
end


for i=1:360
   f(i,1:2)=[fx(i) fy(i)];
end
