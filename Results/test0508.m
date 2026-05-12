clear all
clc

%%%%% initialize %%%%%%%%%%%%%%

my_input.type=2;                                                            %% type=1 slider input; type =2 rotational input
my_input.type1=2;                                                           %% type=1 maximize problem; type1=2 target problem
my_input.max_joint_num=7;                                                   %% maximum joint number allowed

my_input.output=4;                                                          %% output node ID
my_input.output_point=[30,15];                                              %% output node initial position
my_input.endpoint=[30 16;30 18;30 20;30 22];                                %% output node destination
my_input.output_direction=1;      %%%還沒用到                               %% Type1=2 only, Determine output direction + or -

% my_input.constraint0=[1 0 0];     %%%還沒用到                               %% predifined constraint applied
my_input.input=2;                                                              %% input node
my_input.inputlink=1;                                                          %% input link
my_input.fixednode=1;                                                          %% fixed node
my_input.input_point=[15 20];                                              %% input node initial position


my_para.population_size   = 50;    % population size (10 for test, 50 for normal, 100 for accurate)

my_para.iteration_num     = 500;   % number of generation (100 default)
my_para.iteration         = 10;    % cutting into n pieces in solver (10 for fast, 100 for accurate)

my_para.crossover_rate    = 0.6;
my_para.mutation_rate     = 0.02;
my_para.offspringrate     = 0.3;  %% the percentage of offspring position to the whole population

my_para.chromosome_length = 10;
my_para.linklength        = my_input.max_joint_num*(my_input.max_joint_num-1)/2;

my_para.cond_No_break     = 10^4;
my_para.id_break          = 10^(-4);

%%%%% initialize %%%%%%%%%%%%%%

summat=0;
for i=1:my_para.population_size
    summat=summat+i;
end
for i=1:my_para.population_size
    rank(i)=(my_para.population_size+1-i)/summat;
end
cal_sum=cumsum(rank); % cumulative sum of rank vector


k=0;
h=0;
for i=1:my_para.linklength
    my_data.point_num(i,1)=i;
end
for i=1:my_input.max_joint_num-1                       
    for j=1:i
        my_data.point_num(k+j,2)=j;
    end
    k=k+i;
end
for i=1:my_input.max_joint_num-1
    for j=1:i
        my_data.point_num(h+j,3)=i+1;
    end
    h=h+j;
end


%%%%%%
my_data.feasi=0;

while my_data.feasi==0
%%%%%%

%%%%%%%%%%%% linkscript  %%%%%%%%%%%%%%%%%%

%my_data.chromosome=round(rand(1 , 2*(my_para.linklength))); 
my_data.chromosome=round(rand(1 , 3*(my_para.linklength))); 

my_data.link=zeros(1,3);  
%%%%確保有link 產生
if sum(my_data.chromosome(1:my_para.linklength))==0                                          %% in case there is no link at all
    mut=ceil(rand(1)*my_para.linklength);
    my_data.chromosome(mut)=1;
end


if my_input.type==2                                                                   %% make sure input link exist
    my_data.chromosome(1,my_input.inputlink)=1;
    my_data.chromosome(1,my_input.inputlink+my_para.linklength)=0;
    my_data.chromosome(1,my_para.linklength-my_input.max_joint_num+2)=1;
    my_data.chromosome(1,my_para.linklength-my_input.max_joint_num+2+my_para.linklength)=1;    
    my_data.chromosome(1,my_para.linklength-my_input.max_joint_num+2+2*my_para.linklength)=1;    
end


g=0;  
gg=0;
for i=1:my_para.linklength
    if my_data.chromosome(1,i)==1 && my_data.chromosome(1,my_para.linklength+i)==1 && my_data.chromosome(1,2*my_para.linklength+i)==1
        gg=gg+1;
        my_data.fixedlink(gg,:)=my_data.point_num(i,:);
        my_data.chromosome(1,i)=0;
    end
end

%%%%%% constraint decode
my_data.fixednode=zeros(1,my_input.max_joint_num);


for  i=1:size(my_data.fixedlink,1)
     my_data.fixednode(my_data.fixedlink(i,2))=1;
     my_data.fixednode(my_data.fixedlink(i,3))=1;
end


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

%%%%%%%%%%%%connectivity check 


my_data.freenode=zeros(1,my_input.max_joint_num);
my_data.joint_connect=zeros(1,my_input.max_joint_num); % local variable
link1=my_data.link; % local variable
qqqq=0;
temp=0;
my_data.feasi=1;

if my_data.fixednode(my_input.input)==1|| my_data.fixednode(my_input.output)==1
    my_data.feasi=0;
end



if  my_data.fixednode(my_input.input)==0
    for i=1:my_input.max_joint_num
        for j=1:size(link1,1)
                for k=2:3
                    if link1(j,k)==i
                        my_data.joint_connect(i)=my_data.joint_connect(i)+1;
                    end
                end
         end
    end

    for i=1:my_input.max_joint_num
        if my_data.joint_connect(i)==1 && my_data.fixednode(i)==0
            qqqq=1;
            my_data.feasi=0;
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


if my_data.link==0
    my_data.DoF=0;
else
    joint_num=0;
    for i=1:my_input.max_joint_num
        if my_data.fixednode(i)==1
            joint_num=joint_num+my_data.joint_connect(i);
        else
            if my_data.joint_connect(i)>0
                joint_num=joint_num+my_data.joint_connect(i)-1;
            end    
        end    
    end    
    link_num=size(my_data.link,1);    
    my_data.DoF=3*link_num-2*joint_num;       
end

end
