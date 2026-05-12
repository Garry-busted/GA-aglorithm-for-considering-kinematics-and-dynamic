function cal_sum = selection_rank(my_para)

% To calculate the probability of being chosen based on ranking
summat=0;
for i=1:my_para.population_size
    summat=summat+i;
end
for i=1:my_para.population_size
    rank(i)=(my_para.population_size+1-i)/summat;
end
cal_sum=cumsum(rank); % cumulative sum of rank vector
