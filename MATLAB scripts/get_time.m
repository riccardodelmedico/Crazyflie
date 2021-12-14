function [new_time, first_index, size] = get_time(cust_time, int_time)
% GET_TIME returns a new vector 'new_time' which contains the 'int_time' 
% elements that are included between 'cust_time' extremities.
% Also returns the first index and the number of those elements so that they
% can be used to select the correct elements in internal drone data. 

begin_time = cust_time(1);
end_time = cust_time(end);
size = 0;
first_index = 0;
first_iter = true;

for i = 1:length(int_time)
    if(isbetween(int_time(i), begin_time, end_time))
        if(first_iter == true)
            first_index = i;
            first_iter = false;
        end
        size = size + 1;
    end
end
new_time = int_time(first_index:first_index + size - 1);
end

