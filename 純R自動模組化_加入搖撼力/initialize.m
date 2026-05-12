function my_para = initialize(my_input)
my_para.population_size   = 60;    % population size (10 for test, 50 for normal, 100 for accurate)
my_para.iteration_num     = 500;   % number of generation (100 default)
my_para.iteration         = 20;    % cutting into n pieces in solver (10 for fast, 100 for accurate)

my_para.crossover_rate    = 0.6;
my_para.mutation_rate     = 0.08;
my_para.offspringrate     = 0.3;  %% the percentage of offspring position to the whole population

my_para.chromosome_length = 10;
my_para.linklength        = my_input.max_joint_num*(my_input.max_joint_num-1)/2;

my_para.cond_No_break     = 10^4;
my_para.id_break          = 10^(-4);

my_para.sf                = max([my_input.drawing_y(2)-my_input.drawing_y(1)], ...
                                [my_input.drawing_x(2)-my_input.drawing_x(1)]) ...
                                /40; % size factor % default = 40, bigger value gives smaller drawing objects

E=29.5*10^6;
A=1;
my_para.EA=E*A;


