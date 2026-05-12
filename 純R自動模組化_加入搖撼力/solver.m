function [my_data, my_plot] = solver(my_para,my_input,my_data,my_plot)

tree=my_data.joint_connect;
yy=my_data.point;
tree2=zeros(1,my_input.max_joint_num);
tree3=zeros(1,size(my_data.link,1));
joint_position=zeros(my_input.max_joint_num,3);
joint_velocity=zeros(my_input.max_joint_num,3);
joint_acc=zeros(my_input.max_joint_num,3);
link_para=zeros(my_data.link(1),5);
Inverse_count=0;
RR_model=0;
my_data.final_angle=0;
zzzz=0;
% final_joint_position=z
% eros(my_input.max_joint_num,720);
% final_joint_velocity=zeros(my_input.max_joint_num,720);
% final_joint_acc=zeros(my_input.max_joint_num,720);
%%%%將fixednode的位置速度加速度定義出來，並以tree2等於1代表此點為fixednode且資訊皆齊全
for i=1:my_input.max_joint_num
    if my_data.fixednode(i)==1&&my_data.joint_connect(i)>0
        joint_position(i,1)=my_data.point(i,1);
        joint_position(i,2)=my_data.point(i,2);
        joint_position(i,3)=my_data.point(i,3);
        joint_velocity(i,1)=i;
        joint_velocity(i,2)=0;
        joint_velocity(i,3)=0;
        joint_acc(i,1)=i;
        joint_acc(i,2)=0;
        joint_acc(i,3)=0;
        tree(i)=0;
        tree2(i)=1;
    end
end


%%%%將各連桿編號、長度、角度、角速度、角加速度放入link_para之中，並將input給的資訊輸入至input link 中
for i=1:size(my_data.link,1)
    if my_data.link(i,1)==my_input.inputlink
        link_para(i,1)=my_data.link(i,1);
        link_para(i,4)=my_input.input_angularvelocity;
        link_para(i,5)=my_input.input_angularacc;
        xtemp=my_data.point(my_data.link(i,3),2)-my_data.point(my_data.link(i,2),2);
        ytemp=my_data.point(my_data.link(i,3),3)-my_data.point(my_data.link(i,2),3);
        link_para(i,2)=(xtemp^2+ytemp^2)^(1/2);
        link_para(i,3)=atan2(ytemp,xtemp);
        tree3(i)=1;
    else
        link_para(i,1)=my_data.link(i,1);
        xtemp=my_data.point(my_data.link(i,3),2)-my_data.point(my_data.link(i,2),2);
        ytemp=my_data.point(my_data.link(i,3),3)-my_data.point(my_data.link(i,2),3);
        %%%%%不一定對
        link_para(i,2)=(xtemp^2+ytemp^2)^(1/2);
        % link_para(i,3)=atan2(ytemp,xtemp);
    end
end

%%% 測試此機構是否能用FRR或IRR求出
tree_temp=tree;
tree2_temp=tree2;
tree3_temp=tree3;

for j=1:size(my_data.link,1)
    if tree3(j)==1&&tree2(my_data.link(j,2))==1
        Xq=joint_position(my_data.link(j,2),2);
        Yq=joint_position(my_data.link(j,2),3);
        Xq_d=joint_velocity(my_data.link(j,2),2);
        Yq_d=joint_velocity(my_data.link(j,2),3);
        Xq_dd=joint_acc(my_data.link(j,2),2);
        Yq_dd=joint_acc(my_data.link(j,2),3);            
        phi=link_para(j,3);
        phi_d=link_para(j,4);
        phi_dd=link_para(j,5);
        [Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd] =...
        Forward_RR(link_para(j,2),0, Xq,Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,phi, 0,phi_d,0, phi_dd, 0);
        joint_position(my_data.link(j,3),1)=my_data.link(j,3);
        joint_position(my_data.link(j,3),2)=Xa;
        joint_position(my_data.link(j,3),3)=Ya;           
        joint_velocity(my_data.link(j,3),1)=my_data.link(j,3);
        joint_velocity(my_data.link(j,3),2)=Xa_d;
        joint_velocity(my_data.link(j,3),3)=Ya_d;
        joint_acc(my_data.link(j,3),1)=my_data.link(j,3);
        joint_acc(my_data.link(j,3),2)=Xa_dd;
        joint_acc(my_data.link(j,3),3)=Ya_dd;
        tree2_temp(my_data.link(j,3))=1;
        tree_temp(my_data.link(j,3))=0;
    end
end
Inverse_num=zeros(1,my_input.max_joint_num);

kkkk=1;
while kkkk==1
    test=0;
    D=0;
    %%%%要去找與其連接之接頭有沒有兩個已知所有資訊
    for j=1:my_input.max_joint_num
        if tree_temp(j)~=0
            kk=0;
            temp_joint=zeros(1,my_input.max_joint_num);
            temp_length=zeros(1,my_data.link(1));
            for k=1:size(my_data.link,1)
                if my_data.link(k,2)==j && tree_temp(my_data.link(k,3))==0
                    kk=kk+1;
                    temp_joint(kk)=my_data.link(k,3);
                    temp_length(kk)=link_para(k,2);
                elseif my_data.link(k,3)==j && tree_temp(my_data.link(k,2))==0 
                    kk=kk+1;
                    temp_joint(kk)=my_data.link(k,2);
                    temp_length(kk)=link_para(k,2);
                else
                    kk=kk+0;
                end
            end 
            if kk>1
                test=1;
                c=temp_length(1);
                b=temp_length(2);
                Xq=joint_position(temp_joint(1),2);
                Yq=joint_position(temp_joint(1),3);
                Xq_d=joint_velocity(temp_joint(1),2);
                Yq_d=joint_velocity(temp_joint(1),3);
                Xq_dd=joint_acc(temp_joint(1),2);
                Yq_dd=joint_acc(temp_joint(1),3);
                Xa=joint_position(temp_joint(2),2);
                Ya=joint_position(temp_joint(2),3);
                Xa_d=joint_velocity(temp_joint(2),2);
                Ya_d=joint_velocity(temp_joint(2),3);
                Xa_dd=joint_acc(temp_joint(2),2);
                Ya_dd=joint_acc(temp_joint(2),3);
                temp_position=zeros(2,2);
                temp_error=zeros(2,1);
                X = Xa - Xq; Y = Ya - Yq;
                A = 2 * c * Y;
                B = 2 * c * X;
                C = c^2 + X^2 + Y^2 - b^2;
                D=(A^2 + B^2 - C^2);
                if D<0
                    break;
                end    
                if D>0
                %%%%%%在此加入判斷是否超出的程式
                for ii=1:2
                    [phi, psi, phi_d, psi_d, phi_dd, psi_dd] = Inverse_RR(c,b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd,ii);
                    temp_position(ii,1)=Xq+c*cos(phi);
                    temp_position(ii,2)=Yq+c*sin(phi);
                    temp_error(ii,1)=my_data.point(j,2)-temp_position(ii,1)+my_data.point(j,3)-temp_position(ii,2);
                end
                if abs(temp_error(1,1))>=abs(temp_error(2,1))
                    Inverse_count=Inverse_count+1;
                    inverse_num(Inverse_count)=2;
                    [phi, psi, phi_d, psi_d, phi_dd, psi_dd] = Inverse_RR(c,b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd,inverse_num(Inverse_count));
                    joint_position(j,1)=j;
                    joint_position(j,2)=Xq+c*cos(phi);
                    joint_position(j,3)=Yq+c*sin(phi);
                    joint_velocity(j,1)=j;
                    joint_velocity(j,2)=Xq_d-c*phi_d*sin(phi);
                    joint_velocity(j,3)=Yq_d+c*phi_d*cos(phi);
                    joint_acc(j,1)=j;
                    joint_acc(j,2)=Xq_dd-c*phi_dd*sin(phi)-c*phi_d*phi_d*cos(phi);
                    joint_acc(j,3)=Yq_dd+c*phi_dd*cos(phi)-c*phi_d*phi_d*sin(phi);
                    tree2_temp(j)=1;
                    tree_temp(j)=0;
                else
                    Inverse_count=Inverse_count+1;
                    inverse_num(Inverse_count)=1;
                    [phi, psi, phi_d, psi_d, phi_dd, psi_dd] = Inverse_RR(c,b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd,inverse_num(Inverse_count));
                    joint_position(j,1)=j;
                    joint_position(j,2)=Xq+c*cos(phi);
                    joint_position(j,3)=Yq+c*sin(phi);
                    joint_velocity(j,1)=j;
                    joint_velocity(j,2)=Xq_d-c*phi_d*sin(phi);
                    joint_velocity(j,3)=Yq_d+c*phi_d*cos(phi);
                    joint_acc(j,1)=j;
                    joint_acc(j,2)=Xq_dd-c*phi_dd*sin(phi)-c*phi_d*phi_d*cos(phi);
                    joint_acc(j,3)=Yq_dd+c*phi_dd*cos(phi)-c*phi_d*phi_d*sin(phi);
                    tree2_temp(j)=1;
                    tree_temp(j)=0;
                end  
                end
            end    
        end    
    end
    if D<0
       RR_model=1;
        break;
    end
    uuu=0;
    for iii=1:my_input.max_joint_num
        if tree_temp(iii)==0
            uuu=uuu+1;
        end
    end
    if uuu==my_input.max_joint_num
        kkkk=0;
    end    
    if test==0 && kkkk~=0
       RR_model=1;
        break;
    end
end    

%%%
if RR_model==0
    for i=1:360
        qqqq=i;
        i=(i-1)*pi/180;
        tree_temp=tree;
        tree2_temp=tree2;
            
        for j=1:size(my_data.link,1)
            if tree3(j)==1&&tree2(my_data.link(j,2))==1
                Xq=joint_position(my_data.link(j,2),2);
                Yq=joint_position(my_data.link(j,2),3);
                Xq_d=joint_velocity(my_data.link(j,2),2);
                Yq_d=joint_velocity(my_data.link(j,2),3);
                Xq_dd=joint_acc(my_data.link(j,2),2);
                Yq_dd=joint_acc(my_data.link(j,2),3);            
                phi=link_para(j,3);
                phi_d=link_para(j,4);
                phi_dd=link_para(j,5);
                [Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd] =...
                Forward_RR(link_para(j,2),0, Xq,Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,phi-i, 0,phi_d,0, phi_dd, 0);
                joint_position(my_data.link(j,3),1)=my_data.link(j,3);
                joint_position(my_data.link(j,3),2)=Xa;
                joint_position(my_data.link(j,3),3)=Ya;           
                joint_velocity(my_data.link(j,3),1)=my_data.link(j,3);
                joint_velocity(my_data.link(j,3),2)=Xa_d;
                joint_velocity(my_data.link(j,3),3)=Ya_d;
                joint_acc(my_data.link(j,3),1)=my_data.link(j,3);
                joint_acc(my_data.link(j,3),2)=Xa_dd;
                joint_acc(my_data.link(j,3),3)=Ya_dd;
                tree2_temp(my_data.link(j,3))=1;
                tree_temp(my_data.link(j,3))=0;
            end
        end
        
        
        kkkk=1;
        while kkkk==1
            test=0;
            count=0;
            %%%%要去找與其連接之接頭有沒有兩個已知所有資訊
            for j=1:my_input.max_joint_num
                if tree_temp(j)~=0
                    kk=0;
                    temp_joint=zeros(1,my_input.max_joint_num);
                    temp_length=zeros(1,my_data.link(1));
                    for k=1:size(my_data.link,1)
                        if my_data.link(k,2)==j && tree_temp(my_data.link(k,3))==0
                            kk=kk+1;
                            temp_joint(kk)=my_data.link(k,3);
                            temp_length(kk)=link_para(k,2);
                        elseif my_data.link(k,3)==j && tree_temp(my_data.link(k,2))==0 
                            kk=kk+1;
                            temp_joint(kk)=my_data.link(k,2);
                            temp_length(kk)=link_para(k,2);
                        else
                            kk=kk+0;
                        end
                    end 
                    if kk>1
                        test=1;
                        c=temp_length(1);
                        b=temp_length(2);
                        Xq=joint_position(temp_joint(1),2);
                        Yq=joint_position(temp_joint(1),3);
                        Xq_d=joint_velocity(temp_joint(1),2);
                        Yq_d=joint_velocity(temp_joint(1),3);
                        Xq_dd=joint_acc(temp_joint(1),2);
                        Yq_dd=joint_acc(temp_joint(1),3);
                        Xa=joint_position(temp_joint(2),2);
                        Ya=joint_position(temp_joint(2),3);
                        Xa_d=joint_velocity(temp_joint(2),2);
                        Ya_d=joint_velocity(temp_joint(2),3);
                        Xa_dd=joint_acc(temp_joint(2),2);
                        Ya_dd=joint_acc(temp_joint(2),3);
                        X = Xa - Xq; Y = Ya - Yq;
                        A = 2 * c * Y;
                        B = 2 * c * X;
                        C = c^2 + X^2 + Y^2 - b^2;
                        D=(A^2 + B^2 - C^2);
                        if (c+b)>=(((Xq-Xa)^2+(Yq-Ya)^2)^(1/2))&&abs(c-b)<=(((Xq-Xa)^2+(Yq-Ya)^2)^(1/2))&&D>0
                            count=count+1;
                            [phi, psi, phi_d, psi_d, phi_dd, psi_dd] = Inverse_RR(c,b, Xq, Yq,Xq_d,Yq_d,Xq_dd,Yq_dd,Xa,Ya,Xa_d,Ya_d,Xa_dd,Ya_dd,inverse_num(count));
                            joint_position(j,1)=j;
                            joint_position(j,2)=Xq+c*cos(phi);
                            joint_position(j,3)=Yq+c*sin(phi);
                            joint_velocity(j,1)=j;
                            joint_velocity(j,2)=Xq_d-c*phi_d*sin(phi);
                            joint_velocity(j,3)=Yq_d+c*phi_d*cos(phi);
                            joint_acc(j,1)=j;
                            joint_acc(j,2)=Xq_dd-c*phi_dd*sin(phi)-c*phi_d*phi_d*cos(phi);
                            joint_acc(j,3)=Yq_dd+c*phi_dd*cos(phi)-c*phi_d*phi_d*sin(phi);
                            tree2_temp(j)=1;
                            tree_temp(j)=0;
                            my_data.final_angle=qqqq;
                        else
                            my_data.final_angle=qqqq-1;
                            zzzz=1;
                            break;
                        end
                    end    
                end    
            end
            if zzzz==1
                break;
            end    
            uuu=0;
            for iii=1:my_input.max_joint_num
                if tree_temp(iii)==0
                    uuu=uuu+1;
                end
            end
            if uuu==my_input.max_joint_num
                kkkk=0;
            end    

        end 
        if zzzz==1
            break
        else
            my_data.final_joint_position(1:my_input.max_joint_num,my_data.final_angle*2-1:my_data.final_angle*2)=joint_position(1:my_input.max_joint_num,2:3);
            my_data.final_joint_velocity(1:my_input.max_joint_num,my_data.final_angle*2-1:my_data.final_angle*2)=joint_velocity(1:my_input.max_joint_num,2:3);
            my_data.final_joint_acc(1:my_input.max_joint_num,my_data.final_angle*2-1:my_data.final_angle*2)=joint_acc(1:my_input.max_joint_num,2:3);
        end

    end
end    

%% 寫入連桿長度
my_data.num=0;
for i = 1:my_input.max_joint_num
    if my_data.fixednode(i)==1 && my_data.joint_connect(i)>=1
        my_data.num=my_data.num+1;
    end
end
my_data.fixedpoint = zeros(my_data.num, 3);
% ground
jjjjjjj=0;
for i = 1:my_input.max_joint_num
    if my_data.fixednode(i)==1 && my_data.joint_connect(i)>=1
        jjjjjjj = jjjjjjj+1;
        temp_fixed_f = [my_data.point(i,2), my_data.point(i,3)];
        my_data.fixedpoint(jjjjjjj,1)=i;
        my_data.fixedpoint(jjjjjjj,2)=temp_fixed_f(1); 
        my_data.fixedpoint(jjjjjjj,3)=temp_fixed_f(2);
        
    end
end

for i = 1:my_data.num-1
    for j=i+1:my_data.num
        xtemp=my_data.fixedpoint(j,2)-my_data.fixedpoint(i,2);
        ytemp=my_data.fixedpoint(j,3)-my_data.fixedpoint(i,3);
        
    end
    my_data.link_length(i,2) = sqrt(xtemp^2+ytemp^2);
end

for i = 1:size(my_data.link, 1)
     xtemp=my_data.point(my_data.link(i,3), 2)-my_data.point(my_data.link(i,2), 2);
     ytemp=my_data.point(my_data.link(i,3), 3)-my_data.point(my_data.link(i,2), 3);
            
     my_data.link_length(i+my_data.num-1,3) = sqrt(xtemp^2+ytemp^2);
end


%% 搖撼力計算 (修正版) %%

dyn_num_step = my_data.final_angle; 

Shaking_force_history = zeros(1, dyn_num_step);

% 定義誤差
length_tolerance = 1e-4; 

for t = 1 : dyn_num_step
    temp_F_sum_x = 0;
    temp_F_sum_y = 0;
    
    col_x = t*2 - 1;
    col_y = t*2;
    
    Acc_x_all = my_data.final_joint_acc(:, col_x);
    Acc_y_all = my_data.final_joint_acc(:, col_y);
    
    % 判斷這一瞬間是否為有效解
    is_step_valid = true; 
    
    for i = 1 : size(my_data.link, 1)

        node_start = my_data.link(i, 2);
        node_end   = my_data.link(i, 3);
        
        xtemp = my_data.final_joint_position(node_end, col_x) - my_data.final_joint_position(node_start, col_x);
        ytemp = my_data.final_joint_position(node_end, col_y) - my_data.final_joint_position(node_start, col_y);
        current_L = sqrt(xtemp^2 + ytemp^2);
        
        if t == 1
            my_data.L_link(1, i) = current_L; 
        else
            ref_L = my_data.L_link(1, i);
            if abs(current_L - ref_L) > length_tolerance
                fprintf('Error at Angle %d: Link %d length changed! Ref: %.4f, Now: %.4f\n', ...
                        t, i, ref_L, current_L);
                is_step_valid = false;
                break; 
            end
        end
        
        A_s = [Acc_x_all(node_start), Acc_y_all(node_start)];
        A_f = [Acc_x_all(node_end), Acc_y_all(node_end)];
        
        if norm(A_s)>0 || norm(A_f)>0
            m_link = my_data.L_link(1, i) * my_input.rho;
            
            A_com = (A_f + A_s) / 2;
            F_inertia = -m_link * A_com;
            
            temp_F_sum_x = temp_F_sum_x + F_inertia(1);
            temp_F_sum_y = temp_F_sum_y + F_inertia(2);
        end
    end
    
    if ~is_step_valid
        fprintf('Mechanism broken at step %d. Stopping dynamics calculation.\n', t);
        Shaking_force_history(t:end) = NaN; 
        my_data.final_angle = t - 1;
    end
    
    Shaking_force_history(t) = sqrt(temp_F_sum_x^2 + temp_F_sum_y^2);
    my_data.final_shaking_force(1, t) = Shaking_force_history(t);
end


%%
if ~exist('my_data.fitnessvalue', 'var')
    my_data.fitnessvalue = 100000;
end
%%% 寫適應值計算

target_position=zeros(my_data.final_angle,2);
fitness=0;

if RR_model==1
    my_data.fitnessvalue=100000;
end  
if my_data.final_angle~=0
    if RR_model==0
        for ii=1:my_data.final_angle
            target_position(ii,1:2)=my_data.final_joint_position(my_input.output,2*ii-1:2*ii);
        end
        my_data.fitnessvalue=0;
        for i=1:size(my_input.endpoint,1)
            temp_fitness=zeros(my_data.final_angle,1);
            for j=1:my_data.final_angle
                temp_fitness(j)=(target_position(j,1)-my_input.endpoint(i,1))^2+(target_position(j,2)-my_input.endpoint(i,2))^2;
            end
            fitness=min(temp_fitness);
            my_data.fitnessvalue=my_data.fitnessvalue+fitness;
        end    
    end
end



disp(['Population = ' num2str(my_data.ii) '  Fitness Value = ' num2str(my_data.fitnessvalue)]);





%%%% 繪圖
my_plot = [];
% if my_data.fitnessvalue<0.00065
if my_data.zz == 500
    for iiiii=1:my_input.max_joint_num
        my_data.good_position=my_data.final_joint_position(:,1:2);
    end
    for kkkk=1:100000
        if exist(['position_no',num2str(kkkk,'%03.f'),'.mat'],'file')==0           % if i'th result not exist
            iterative_filename=['position_no',num2str(kkkk-1,'%03.f'),'.mat'];
            save(iterative_filename,'my_data');
            break;
        end       
    end
    if isequal(my_plot,[])
        my_plot.h_fig = figure;
        set(gcf,'color','white','position',[50 50 800 800]);
        xlim([my_input.drawing_x(1) my_input.drawing_x(2)]);
        ylim([my_input.drawing_y(1) my_input.drawing_y(2)]);


        axis equal;
        V=[my_input.drawing_x(1),my_input.drawing_x(2), ...
              my_input.drawing_y(1),my_input.drawing_y(2)];
        axis(V);

        for p=1:my_data.final_angle
            xlim([my_input.drawing_x(1) my_input.drawing_x(2)]);
            ylim([my_input.drawing_y(1) my_input.drawing_y(2)]);

            cla;
            for j=1:p
                plot(0.2*cos(0 : 0.01 : 2*pi)+my_data.final_joint_position(my_input.output,2*j-1), 0.2*sin( 0: 0.01 : 2*pi)+my_data.final_joint_position(my_input.output,2*j),'r','LineWidth',1); 
            end    
            for i=1:3
                plot(0.5*cos(0 : 0.01 : 2*pi)+my_input.endpoint(i,1), 0.5*sin( 0: 0.01 : 2*pi)+my_input.endpoint(i,2),'k','LineWidth',2); 
               hold on
            end
            for pl=1:size(my_data.link,1)
                plot([my_data.final_joint_position(my_data.link(pl,2),2*p-1),my_data.final_joint_position(my_data.link(pl,3),2*p-1)],...
                    [my_data.final_joint_position(my_data.link(pl,2),2*p),my_data.final_joint_position(my_data.link(pl,3),2*p)],'k','LineWidth',2);

            end  
            pause(0.05);

        end    

    end
    figure(my_plot.h_fig);
    for jjjjjj = 1:100000
        filename = ['position_no', num2str(jjjjjj, '%03.f'), '.bmp'];
        if exist(filename, 'file') == 0  % 如果文件不存在
            iterative_filename = ['position_no', num2str(jjjjjj - 1, '%03.f'), '.bmp'];
            saveas(gcf, iterative_filename, 'bmp');
            break;  % 保存後退出循環
        end
    end
end


























