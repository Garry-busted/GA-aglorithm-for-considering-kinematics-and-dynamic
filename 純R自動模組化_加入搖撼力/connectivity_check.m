
function my_data = connectivity_check(my_input,my_data)


my_data.freenode=zeros(1,my_input.max_joint_num);
my_data.joint_connect=zeros(1,my_input.max_joint_num); % local variable
link1=my_data.link; % local variable
qqqq=0;
temp=0;
my_data.feasi=1;

%%%當輸入joint是 fixed時，其feasible=0
if my_data.fixednode(my_input.input)==1 || my_data.fixednode(my_input.output)==1 
    my_data.feasi=0;
end



if  my_data.fixednode(my_input.input)==0
    %%%計算每個點旁邊有幾個連桿%%%%
    for i=1:my_input.max_joint_num
        for j=1:size(link1,1)
                for k=2:3
                    if link1(j,k)==i
                        my_data.joint_connect(i)=my_data.joint_connect(i)+1;
                    end
                end
         end
    end
    %%%fixednode只有一個代表錯誤
    temp2=0;
    for i=1:my_input.max_joint_num
        if my_data.joint_connect(i)~=0 && my_data.fixednode(i)==1
            temp2=temp2+1;
        end    
    end
    if temp2<2
        my_data.feasi=0;
    end
    
   %%%當free joint旁邊只有一個相鄰桿件代表錯誤
    for i=1:my_input.max_joint_num
        if my_data.joint_connect(i)==1 && my_data.fixednode(i)==0
            qqqq=1;
            my_data.feasi=0;
        end    
    end

    %%%當free joint旁邊有超過兩個相鄰接頭代表不會動，所以錯誤
    if qqqq==0
        for i=1:my_input.max_joint_num
            if my_data.joint_connect(i)~=0 && my_data.fixednode(i)==0
                for j=1:size(link1,1)
                    for k=2:3
                       if link1(j,k)==i
                           if k==2
                               if my_data.fixednode(link1(j,3))==1
                                    temp=temp+1;
                               end 
                           else
                               if my_data.fixednode(link1(j,2))==1
                                     temp=temp+1;
                               end 
                           end    
                       end
                    end   
                end    
            end 
            if temp>1
                my_data.feasi=0; 
            end    
            temp=0;
        end    
    end    
end


tr1=[0,0];
%%%%移除三角形
tr2=1;
if my_data.feasi==1
    for i=1:my_input.max_joint_num
        if my_data.joint_connect(i)==2&&my_data.fixednode(i)==0&& i~=my_input.output
             for j=1:size(link1,1)
                 for k=2:3
                       if link1(j,k)==i
                           if k==2
                                tr1(tr2)=link1(j,3);
                                tr2=tr2+1;
                           else
                                tr1(tr2)=link1(j,2);
                                tr2=tr2+1;
                           end    
                       end
                 end      
             end
             for h=1:size(link1,1)
                 if link1(h,2)==tr1(1)&&link1(h,3)==tr1(2)
                     my_data.joint_connect(i)=0;
                     for q=1:size(link1,1)
                         if link1(q,2)==i||link1(q,3)==i
                             my_data.link(q,:)=[0,0,0];
                         end
                     end
                 end  
                 if link1(h,2)==tr1(2)&&link1(h,3)==tr1(1)
                     my_data.joint_connect(i)=0;
                     for q=1:size(link1,1)
                         if link1(q,2)==i||link1(q,3)==i
                             my_data.link(q,:)=[0,0,0];
                         end
                     end
                 end                  
             end    
        end    
    end    
end

rows_to_delete = all(my_data.link == 0, 2);

% 刪除全部為零的行
my_data.link(rows_to_delete, :) = [];
