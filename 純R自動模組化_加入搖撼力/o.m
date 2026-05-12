clear all
clc 
load('position_no000.mat');
Xo=0;
Yo=0;
X1=10;
Y1=10;
X2=my_data.good_position(4,1);
Y2=my_data.good_position(4,2);
X3=my_data.good_position(6,1);
Y3=my_data.good_position(6,2);

r1=((X1-Xo)^2+(Y1-Yo)^2)^(1/2);
r2=((X2-X1)^2+(Y2-Y1)^2)^(1/2);
r3=((X2-X3)^2+(Y2-Y3)^2)^(1/2);

phi=atan2(Y1,X1);
my_input.endpoint=[40 30;29.6225 28.1480;18.2995 20.7144];

for i=1:360
    [Xa(i),Ya(i),Xa_d(i),Ya_d(i),Xa_dd(i),Ya_dd(i)] = Forward_RR(r1,0, Xo, Yo,0,0,0,0,phi+(i-1)*pi/180,0,5,0, 0,0);
    [t4(i),t3(i), t4_d(i), t3_d(i), t4_dd(i), t3_dd(i)] = Inverse_RR(r3,r2, X3, Y3,0,0,0,0,Xa(i),Ya(i),Xa_d(i),Ya_d(i),Xa_dd(i),Ya_dd(i),2);
    [fx(i),fy(i),fx_d(i),fy_d(i),fx_dd(i),fy_dd(i)] = Forward_RR(r3,0, X3, Y3,0,0,0,0,t4(i),0,t4_d(i),0, t4_dd(i),0);
end

for i=1:360
   f(i,1:2)=[fx(i) fy(i)];
end

figure;
for p=1:1
    cla;
    xlim([-10 50]);
    ylim([-20 40]);
    axis equal;
    hold on;
    for j=1:p
        plot(0.2*cos(0 : 0.01 : 2*pi)+fx(j), 0.2*sin( 0: 0.01 : 2*pi)+fy(j),'r','LineWidth',1);
    end    
    % hold on
    % xlim([-10 50]);
    % ylim([-20 40]);
    % axis equal;
    plot([Xo+0.8*cos(phi+(p-1)*pi/180),Xa(p)-0.8*cos(phi+(p-1)*pi/180)],[Yo+0.8*sin(phi+(p-1)*pi/180),Ya(p)-0.8*sin(phi+(p-1)*pi/180)],'k','LineWidth',2);
    plot([Xa(p)+0.8*cos(t3(p)-pi),fx(p)-0.8*cos(t3(p)-pi)],[Ya(p)+0.8*sin(t3(p)-pi),fy(p)-0.8*sin(t3(p)-pi)],'k','LineWidth',2);
    plot([fx(p)-0.8*cos(t4(p)),X3+0.8*cos(t4(p))],[fy(p)-0.8*sin(t4(p)),Y3+0.8*sin(t4(p))],'k','LineWidth',2);
    plot(0.8*cos(0 : 0.01 : 2*pi)+Xo, 0.8*sin( 0: 0.01 : 2*pi)+Yo,'k','LineWidth',2);
    plot(0.8*cos(0 : 0.01 : 2*pi)+Xa(p), 0.8*sin( 0: 0.01 : 2*pi)+Ya(p),'k','LineWidth',2);
    plot(0.8*cos(0 : 0.01 : 2*pi)+fx(p), 0.8*sin( 0: 0.01 : 2*pi)+fy(p),'k','LineWidth',2);
    plot(0.8*cos(0 : 0.01 : 2*pi)+X3, 0.8*sin( 0: 0.01 : 2*pi)+Y3,'k','LineWidth',2);

    plot(1.5*cos(0 : 0.01 : pi)+Xo, 1.5*sin( 0: 0.01 : pi)+Yo,'k','LineWidth',2);
    plot([Xo-1.5,Xo-1.5],[Yo,Yo-2],'k','LineWidth',2);
    plot([Xo+1.5,Xo+1.5],[Yo,Yo-2],'k','LineWidth',2);
    plot([Xo-2.2,Xo+2.2],[Yo-2,Yo-2],'k','LineWidth',2);
    plot([Xo-2.2,Xo-1.4],[Yo-2.8,Yo-2],'k','LineWidth',2);
    plot([Xo-1.4,Xo-0.6],[Yo-2.8,Yo-2],'k','LineWidth',2);
    plot([Xo-0.6,Xo+0.2],[Yo-2.8,Yo-2],'k','LineWidth',2);
    plot([Xo+0.2,Xo+1],[Yo-2.8,Yo-2],'k','LineWidth',2);
    plot([Xo+1,Xo+1.8],[Yo-2.8,Yo-2],'k','LineWidth',2);

    plot(1.5*cos(0 : 0.01 : pi)+X3, 1.5*sin( 0: 0.01 : pi)+Y3,'k','LineWidth',2);
    plot([X3-1.5,X3-1.5],[Y3,Y3-2],'k','LineWidth',2);
    plot([X3+1.5,X3+1.5],[Y3,Y3-2],'k','LineWidth',2);
    plot([X3-2.2,X3+2.2],[Y3-2,Y3-2],'k','LineWidth',2);
    plot([X3-2.2,X3-1.4],[Y3-2.8,Y3-2],'k','LineWidth',2);
    plot([X3-1.4,X3-0.6],[Y3-2.8,Y3-2],'k','LineWidth',2);
    plot([X3-0.6,X3+0.2],[Y3-2.8,Y3-2],'k','LineWidth',2);
    plot([X3+0.2,X3+1],[Y3-2.8,Y3-2],'k','LineWidth',2);
    plot([X3+1,X3+1.8],[Y3-2.8,Y3-2],'k','LineWidth',2);
    
    % plot(0.6*cos(0 : 0.01 : 2*pi)+my_input.endpoint(1,1), 0.6*sin( 0: 0.01 : 2*pi)+my_input.endpoint(1,2),'b','LineWidth',2);
    % plot(0.6*cos(0 : 0.01 : 2*pi)+my_input.endpoint(2,1), 0.6*sin( 0: 0.01 : 2*pi)+my_input.endpoint(2,2),'b','LineWidth',2);
    % plot(0.6*cos(0 : 0.01 : 2*pi)+my_input.endpoint(3,1), 0.6*sin( 0: 0.01 : 2*pi)+my_input.endpoint(3,2),'b','LineWidth',2);
    if p==1||p==60||p==120||p==360
       for jjjjjj = 1:100000
        filename = ['no', num2str(jjjjjj, '%03.f'), '.png'];
        if exist(filename, 'file') == 0  % 如果文件不存在
            iterative_filename = ['no', num2str(jjjjjj, '%03.f'), '.png'];
            exportgraphics(gcf, iterative_filename, 'Resolution', 300);
            break;  % 保存後退出循環
        end
       end
    end
    pause(0.1);
end    

