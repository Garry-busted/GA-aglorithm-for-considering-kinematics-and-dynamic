function my_data = Degree_of_freedom(my_input,my_data)


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
