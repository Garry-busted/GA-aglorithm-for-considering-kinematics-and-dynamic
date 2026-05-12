function total_population1 = GA_selection(my_para, my_data)

rd=rand(1,my_para.population_size-2);  % size = 48
for i=1:my_para.population_size-2  % 1:48
     if my_data.cal_sum(1)>=rd(i)
        index_num(i)=1;
     else 
         for j=2:my_para.population_size  % 2:50
            if (my_data.cal_sum(j)>=rd(i)) && (my_data.cal_sum(j-1)<=rd(i)) % find the first j that has larger cal_sum(j) than rd(i)
                index_num(i)=j;
            end
         end
     end
end

total_population1(3:my_para.population_size,:)=my_data.total_population(index_num,:);
total_population1=sortrows(total_population1);  %% make the similar ones closer to each other
