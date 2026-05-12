% function GA(handles)
clear all
clc;
close(figure(1));
% profile on -detail builtin -history

%% checking if previous results exist
for iiii=0:500
    if exist(['result_no',num2str(iiii,'%03.f'),'.mat'],'file')==0           % if i'th result not exist
        if exist(['result_no',num2str(iiii-1,'%03.f'),'.mat'],'file')~=0     % if (i-1)'th result exist
            load(['result_no',num2str(iiii-1,'%03.f'),'.mat']);       % load (i-1)'th result, which is the latest result
        end
        my_data.zz=iiii;
        break;
    end
end

clear iiii
%% 當前面沒有數據時則跑下面程式
if my_data.zz==0
    % if get(handles.radiobutton_fourpoint,'value')==1
    %     my_input = model_four_point;
    % elseif get(handles.radiobutton_gripper,'value')==1
    %     my_input = model_gripper;
    % elseif get(handles.radiobutton_inverter,'value')==1
    %     my_input = model_inverter;
    % elseif  get(handles.radiobutton_toggle,'value')==1
    %     my_input = model_toggle;
    % elseif get(handles.radiobutton_twopoint,'value')==1
    %     my_input = model_two_point;
    % elseif get(handles.radiobutton_threepoint,'value')==1
    %     my_input = model_three_point;
    % end

    %%%my_input = model_stright_line;  %%%直線機構測試
    my_input = model_four_point;  %%%直線機構測試
    my_para = initialize(my_input);
    my_plot = [];
    
    my_data.cal_sum = selection_rank(my_para); 
    my_data.point_num = link_script(my_input,my_para);
    [my_data, my_plot] = newpopulation(my_para,my_input,my_data,my_plot);
    my_data = postprocessing(my_data,my_para,my_input);
end

% manually change termination criterion
% my_para.iteration_num     = 500;

% while my_data.zz < my_data.iter_change(size(my_data.iter_change,1),1)+50 && my_data.zz < my_para.iteration_num
% if no better design in 50 continuous iterations then stop, or max
% iteration number is reached
while my_data.zz < my_para.iteration_num
    total_population1 = GA_selection(my_para, my_data);
    total_population1 = GA_offspring(my_input, my_para, total_population1);
    total_population2 = GA_crossover(my_para, my_input, total_population1);                 
    my_data.total_population3 = GA_mutation(my_para, my_input, total_population2);
    if ~exist('my_plot','var'), my_plot=[]; end
    [my_data, my_plot] = newpopulation(my_para,my_input,my_data,my_plot);                   
    my_data = postprocessing(my_data,my_para,my_input);
end

% profile report