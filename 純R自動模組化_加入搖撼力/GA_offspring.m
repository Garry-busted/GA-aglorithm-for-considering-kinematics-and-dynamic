function total_population1 = GA_offspring(my_input, my_para, total_population1)

if my_input.offspring==1
    off_index=rand(1,my_para.population_size);
    for i=3:my_para.population_size
        if off_index(i)<my_para.offspringrate
            v1 = my_para.linklength+my_input.max_joint_num + 1;
            v2 = my_para.linklength+my_input.max_joint_num + 2*my_input.max_joint_num*my_para.chromosome_length;
            temp = rand(1 , 2*my_input.max_joint_num*my_para.chromosome_length);
            total_population1(i,v1:v2) = round(temp);
        end
    end
end
