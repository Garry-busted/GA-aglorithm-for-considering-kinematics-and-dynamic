
function my_data = gener_point(my_para,my_input,my_data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% Binary to Decimal %%%%%%     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

initial_chrom=zeros(1,2 * (my_input.max_joint_num));
for i=1: my_para.chromosome_length
    for j=1:2*(my_input.max_joint_num)
        if my_data.chromosome(1,(my_para.linklength)+(my_input.max_joint_num) +j* (my_para.chromosome_length) +1-i )==1
            initial_chrom(1,j)=initial_chrom(1,j)+2^(i-1);
        end
    end
end
for i=1:my_input.max_joint_num
    initial_position(2*i-1)=(my_input.xrange(2)-my_input.xrange(1))/((2^my_para.chromosome_length)-1).*(initial_chrom(2*i-1)-2^0)+my_input.xrange(1);
    initial_position(2*i)  =(my_input.yrange(2)-my_input.yrange(1))/((2^my_para.chromosome_length)-1).*(initial_chrom(2*i)-2^0)  +my_input.yrange(1);        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% Decode %%%%%%%%%%%%%%     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:my_input.max_joint_num
    my_data.point(i,1)=i;
    my_data.point(i,2)=initial_position(2*i-1);
    my_data.point(i,3)=initial_position(2*i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Predefined Positions %%%%%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isfield(my_input,'output_point')
    my_data.point(my_input.output,:)=[my_input.output,my_input.output_point];
end

if isfield(my_input,'added_point')
    for i=1:size(my_input.added_point,1)
        my_data.point(my_input.added_point(i,1),:)=my_input.added_point(i,:);
    end
end

if isfield(my_input,'input_point')
    my_data.point(my_input.input,:)=[my_input.input,my_input.input_point];
end
if isfield(my_input,'fixed_point')
    my_data.point(my_input.fixednode,:)=[my_input.fixednode,my_input.fixed_point];
end



