function [dataset, goal] = generateRandom2D(dim)

cols=dim(1);
rows=dim(2);
dataset.cols = cols;
dataset.rows = rows;
dataset.cell_size = 0.01;
dataset.origin_x = -rows*dataset.cell_size/2;
dataset.origin_y = -cols*dataset.cell_size/2;

goal_r = 1.2*rand;
goal_th = 2*pi*rand;
goal = goal_r * [cos(goal_th), sin(goal_th)]';

dataset.map = zeros(dataset.rows, dataset.cols);
good_rect = false;
while ~good_rect
    rect1 = random_rect(dataset.rows, dataset.cols, 1, 1);
    rect2 = random_rect(dataset.rows, dataset.cols, 1, 1);
    good_rect = check_goal_collision(goal, rect1, dataset) ...
             && check_goal_collision(goal, rect2, dataset) ...
             && check_back_rect(dataset.rows, dataset.cols, rect1, rect2);
end

% good_rect=false;
% while ~good_rect 
%     rect3 = random_rect(dataset.rows, dataset.cols, 1, 1);
%     good_rect=check_bad_rect(dataset.rows, dataset.cols, rect1, rect3);
%     if good_rect
%         good_rect=check_bad_rect(dataset.rows, dataset.cols, rect2, rect3);
%     end
% end


dataset.map(rect1(1):rect1(2), rect1(3):rect1(4)) = 1;
dataset.map(rect2(1):rect2(2), rect2(3):rect2(4)) = 1;
% dataset.map(rect3(1):rect3(2), rect3(3):rect3(4)) = 1;

end

function rect = random_rect(rows, cols, start_x, start_y)

% Returns [x_bot, x_top, y_left, y_right]

height = randi([20, floor(rows/2)]);
width = randi([20, floor(cols/2)]);
x_bot = randi([start_x, rows-height]);
y_left = randi([start_y, cols-width]);

rect = [x_bot, x_bot+height, y_left, y_left+width];
end

function ok = check_goal_collision(goal, rect, dataset)

goal_img_frame = [(-dataset.origin_x + goal(1))/dataset.cell_size,
                  (-dataset.origin_y + goal(2))/dataset.cell_size];
              
inside_rect = (goal_img_frame(2) >= rect(1) && goal_img_frame(2) <= rect(2) ...
            && goal_img_frame(1) >= rect(3) && goal_img_frame(1) <= rect(4));

ok = ~inside_rect;

end

function ok = check_bad_rect(rows,cols,rect1,rect2)
    
    a1=zeros(rows,cols);
    a1(rect1(1):rect1(2), rect1(3):rect1(4)) = 1;
    
    a2=zeros(rows,cols);
    a2(rect2(1):rect2(2), rect2(3):rect2(4)) = 1;
    
    %disp(size(a1));
    %disp(size(a2));
    
    a=a1+a2;
    
    ok = max(a(:))<=1 ;
    
end