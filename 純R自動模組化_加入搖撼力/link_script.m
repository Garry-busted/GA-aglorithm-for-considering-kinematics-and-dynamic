function output_point_num = link_script(my_input,my_para)

k=0;
h=0;
for i=1:my_para.linklength
    output_point_num(i,1)=i;
end
for i=1:my_input.max_joint_num-1                       
    for j=1:i
        output_point_num(k+j,2)=j;
    end
    k=k+i;
end
for i=1:my_input.max_joint_num-1
    for j=1:i
        output_point_num(h+j,3)=i+1;
    end
    h=h+j;
end
