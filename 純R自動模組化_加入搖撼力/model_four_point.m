function my_input = model_four_point
my_input.type=2;                                                            %% type=1 slider input; type =2 rotational input
my_input.type1=2;                                                           %% type=1 maximize problem; type1=2 target problem
my_input.max_joint_num=6;                                                   %% maximum joint number allowed

my_input.output=4;                                                          %% output node ID
my_input.output_point=[40,30];                                              %% output node initial position
my_input.endpoint=[40 30;29.6225 28.1480;18.2995 20.7144];                 %% output node destination
my_input.output_direction=1;      %%%還沒用到                               %% Type1=2 only, Determine output direction + or -

% my_input.constraint0=[1 0 0];     %%%還沒用到                               %% predifined constraint applied
my_input.input=2;                                                              %% input node
my_input.inputlink=1;                                                          %% input link
my_input.fixednode=1;                                                          %% fixed node

my_input.fixed_point=[0 0];                                              %% input node initial position
my_input.input_point=[10 10];                                              %% input node initial position
my_input.max_input_value=5/180*pi;     %%%還沒用到                          %% maximum input value, Unit: Rad or lenght unit
my_input.input_angularvelocity=5;
my_input.input_angularacc=0;


my_input.penalty_link=0;         %%%還沒用到                                %% penalty method application (abandoned)
my_input.noslider=1;                                                       %% noslider=1 do not allow extra slider , noslider=0 allow extra slider
my_input.offspring=1;            %%%還沒用到                           %% offspring=1 allow the offspring chromosome to enter, offspring =0 do not allow

my_input.rho=0.001;                                                            %% 線密度(kg/unit length)

 %%%還沒用到 
my_input.inside_domain_xl=0;                                                %%  0 ==> allow the final positions of the nodes to pass the domain limit
my_input.inside_domain_xu=0;                                                %%  1 ==> do not allow it to cross the domain limit
my_input.inside_domain_yl=0;                                                %% xl ==> lower x limit   xu ==> upper x limit
my_input.inside_domain_yu=0;                                                %% yl ==> lower y limit   yu ==> upper y limit
my_input.xrange=[-50,50];                                                   %% xrange of the design domain
my_input.yrange=[-50,50];                                                   %% yrange of the design domain
my_input.drawing_x=my_input.xrange;                                         %% x region for animation purpose
my_input.drawing_y=my_input.yrange;                                                          