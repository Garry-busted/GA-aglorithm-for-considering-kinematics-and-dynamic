
function my_data = lk_cons_decode(my_para,my_input,my_data)

my_data.link=zeros(1,3);  

%%%%確保有link 產生
if sum(my_data.chromosome(1:my_para.linklength))==0                                          %% in case there is no link at all
    mut=ceil(rand(1)*my_para.linklength);
    my_data.chromosome(mut)=1;
end

%%%%使得 input link 的連桿產生並且必不為固定桿 
if my_input.type==2                                                                   %% make sure input link exist
    my_data.chromosome(1,my_input.inputlink)=1;
    my_data.chromosome(1,my_para.linklength+my_input.input)=0;
    my_data.chromosome(1,my_para.linklength+my_input.fixednode)=1;
    %my_data.chromosome(1,my_para.linklength+my_input.max_joint_num)=1;
end

g=0;  
gg=0;



% %%%%找出固定桿、將其放置於fixedlink內、將固定之連桿消除
% for i=1:my_para.linklength
%     if my_data.chromosome(1,i)==1 && my_data.chromosome(1,my_para.linklength+i)==1 && my_data.chromosome(1,2*my_para.linklength+i)==1
%         gg=gg+1;
%         my_data.fixedlink(gg,:)=my_data.point_num(i,:);
%         my_data.chromosome(1,i)=0;
%     end
% end

%%%%%% constraint decode

my_data.fixednode=zeros(1,my_input.max_joint_num);

% %%%%%% 當為旋轉接頭，fixednode 1等於fixednode
% if my_input.type==2                                                                  
%     my_data.fixednode(my_input.fixednode)=1;
% 
% end

%%%%%% 將存在fixedlink內的joint 變為fixednode 等於 1
% for  i=1:size(my_data.fixedlink,1)
%      my_data.fixednode(my_data.fixedlink(i,2))=1;
%      my_data.fixednode(my_data.fixedlink(i,3))=1;
% end
for  i=1:my_input.max_joint_num
    if my_data.chromosome(1,my_para.linklength+i)==1
        my_data.fixednode(i)=1;
    else
        my_data.fixednode(i)=0;
    end 
end

%%%%%% 將兩端都為 fixednode時，將連桿消除
for  i=1:my_para.linklength
     if my_data.fixednode(my_data.point_num(i,2))==1 && my_data.fixednode(my_data.point_num(i,3))==1
        my_data.chromosome(1,i)=0;
     end
end


for i=1:my_para.linklength
    if my_data.chromosome(1,i)==1

        g=g+1;
        my_data.link(g,:)=my_data.point_num(i,:);
    end
end    


    