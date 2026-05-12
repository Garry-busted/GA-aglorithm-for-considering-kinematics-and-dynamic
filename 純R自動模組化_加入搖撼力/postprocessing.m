function my_data = postprocessing(my_data,my_para,my_input)

if my_data.zz == 0
    my_data.fitp=my_data.fitness2(1);    % best fitness value
    my_data.final_popu(1,:)=my_data.total_population(1,:);     % best chromosome
    my_data.iter_change(1,:)=[my_data.zz,my_data.fitness2(1)];   % history of best fitness value
    my_data.best2 = my_data.final_popu;
else
    if my_data.fitness2(1)<my_data.fitp   % works for smaller fitness the better
        my_data.best2(2,:) = my_data.best2(1,:);
        my_data.best2(1,:) = my_data.total_population(1,:);
        my_data.fitp=my_data.fitness2(1);
        my_data.final_popu (size(my_data.iter_change,1)+1,:)=my_data.total_population(1,:);
        my_data.iter_change(size(my_data.iter_change,1)+1,:)=[my_data.zz,my_data.fitness2(1)];
    end
end

iterative_filename=['result_no',num2str(my_data.zz,'%03.f'),'.mat'];
save(iterative_filename,'my_data','my_para','my_input');
my_data.zz=my_data.zz+1;

disp('----------');
disp(['Iteration No. ',num2str(my_data.zz)]);
disp(['Best Fitness Value = ',num2str(my_data.fitp)]);
disp('History of Best Fitness Values');
disp(num2str(my_data.iter_change));
disp('Sorting of Fitness Values');
disp(num2str(my_data.fitness2));
disp('----------');

