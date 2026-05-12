function total_population3 = GA_mutation(my_para, my_input, total_population2)
total_population3=total_population2;
v1 = my_para.population_size;
v2 = my_para.linklength+my_input.max_joint_num + 2*my_input.max_joint_num*my_para.chromosome_length;
mutate=rand(v1,v2);
for i=1:v1
    for j=1:v2
        if mutate(i,j)<=my_para.mutation_rate
            if total_population2(i,j)==1
                total_population3(i,j)=0;
            else
                total_population3(i,j)=1;
            end
        else
            total_population3(i,j)=total_population2(i,j);
        end   
    end
end