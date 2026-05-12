function [my_data, my_plot] = newpopulation(my_para,my_input,my_data,my_plot)
    used=1;
    modec = 0;
    while used<my_para.population_size+1
        
        % Generate new Chromosome
        if my_data.zz==0 || modec==1
    
           %%%% old 
           % my_data.chromosome=round(rand(1 , (my_para.linklength) +2* (my_para.chromosome_length) ...
           %                           *(my_input.max_joint_num) +2* (my_input.max_joint_num)    )); 
    
           %%%% new only R joint (對連桿去約束是否為地桿，且由於只有R joint所以不需要其餘 joint constrain
           my_data.chromosome=round(rand(1 ,(my_para.linklength) +2* (my_para.chromosome_length) ...
                                     *(my_input.max_joint_num)+(my_input.max_joint_num))); 
           modec=0;
        else 
           my_data.chromosome=my_data.total_population3(used,:);
        end
      
        % feasibility Check
        my_data = lk_cons_decode(my_para,my_input,my_data);
        my_data = connectivity_check(my_input,my_data);
        my_data = Degree_of_freedom(my_input,my_data);
    
        %Create population pool  
        if my_data.feasi==0 || my_data.DoF>1 || my_data.DoF<1 ||size(my_data.link,1)>4
            modec=1;
        else
            my_data.total_population(used,:)=my_data.chromosome;
            used=used+1;
        end
    end
    
    % Elitism
    % if isfield(my_data, 'iter_change')
    %     my_data.total_population(1:size(my_data.best2,1),:) = my_data.best2;
    % end
    if my_data.zz~=0
        my_data.total_population(1:size(my_data.best2,1),:) = my_data.best2;
    end
    
    % Gain fitnessvalue
    for ii=1:my_para.population_size
         my_data.ii = ii;
         
         % 👑 菁英快取防護機制：直接沿用上一代鎖定的成績，跳過解碼與 Solver 緩衝區寫入
         if my_data.zz~=0 && ii<=size(my_data.best2,1)
             my_data.fitness2(1,ii) = my_data.best_fitness_cache(ii);
             continue;
         end
         
         % 一般平民分子：乖乖執行標準解碼與解算流程
         my_data.chromosome=my_data.total_population(my_data.ii,:);
         my_data = lk_cons_decode(my_para,my_input,my_data);
         my_data = connectivity_check(my_input,my_data);
         my_data = Degree_of_freedom(my_input,my_data);
         my_data = gener_point(my_para,my_input,my_data);
         % [my_data, my_plot] = solver(my_para,my_input,my_data,my_plot);
         [my_data, my_plot] = solver_multi_object(my_para,my_input,my_data,my_plot);
         my_data.fitness2(1,my_data.ii)=my_data.fitnessvalue;
    end
    
    % sorting population wrt its fitnessvalue
    [my_data.fitness2,index2]=sort(my_data.fitness2,'ascend');  % sort all population wrt fitness ranking, small 2 big, good 2 bad
    population_temp = my_data.total_population(index2,:);   
    my_data.total_population = population_temp;
    
    % 💾 更新菁英快取 (供下一代迴圈開頭跳過運算使用)
    my_data.best2 = my_data.total_population(1:2, :);
    my_data.best_fitness_cache = my_data.fitness2(1, 1:2);
    
    % 📸 額外存儲當代最優解數據快照 (Snapshot)
    % 單獨提取當代排完序後的第一名重新運算，並將完整軌跡獨立存儲，避免干擾主運算邏輯
    my_data.ii = 1;
    my_data.chromosome = my_data.total_population(1, :);
    my_data = lk_cons_decode(my_para, my_input, my_data);
    my_data = connectivity_check(my_input, my_data);
    my_data = Degree_of_freedom(my_input, my_data);
    my_data = gener_point(my_para, my_input, my_data);
    
    [my_data, my_plot] = solver_multi_object(my_para, my_input, my_data, my_plot);
    
    % 將冠軍專屬數據獨立搬家至獨立子欄位
    my_data.best_individual.shaking_force = my_data.final_shaking_force;
    my_data.best_individual.position      = my_data.final_joint_position;
    my_data.best_individual.velocity      = my_data.final_joint_velocity;
    my_data.best_individual.acc           = my_data.final_joint_acc;
    my_data.best_individual.fitness       = my_data.best_fitness_cache(1);
    my_data.best_individual.chromosome    = my_data.total_population(1, :);
    
    % 確保最外層回傳的單一適應值嚴格對齊快取的精確值
    my_data.fitnessvalue = my_data.best_fitness_cache(1);

% Gain fitnessvalue
% for ii=1:my_para.population_size
%      my_data.ii = ii;
%      my_data.chromosome=my_data.total_population(my_data.ii,:);
%      my_data = lk_cons_decode(my_para,my_input,my_data);
%      my_data = connectivity_check(my_input,my_data);
%      my_data = Degree_of_freedom(my_input,my_data);
%      my_data = gener_point(my_para,my_input,my_data);
%      % [my_data, my_plot] = solver(my_para,my_input,my_data,my_plot);
%      [my_data, my_plot] = solver_multi_object(my_para,my_input,my_data,my_plot);
%      my_data.fitness2(1,my_data.ii)=my_data.fitnessvalue;
% end