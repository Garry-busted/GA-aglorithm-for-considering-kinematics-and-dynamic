function total_population2 = GA_crossover(my_para, my_input, total_population1)

v1 = my_para.population_size/2;
v2 = my_para.linklength+my_input.max_joint_num + 2*my_input.max_joint_num*my_para.chromosome_length;
cross_index=rand(v1,v2);
total_population2=total_population1;
for i=1:v1
    for j=1:v2
        if cross_index(i,j)<=my_para.crossover_rate
            total_population1=total_population2;
            total_population2(2*i-1,1:j)=total_population1(2*i,1:j);
            total_population2(2*i,1:j)=total_population1(2*i-1,1:j);
        end
    end
end


