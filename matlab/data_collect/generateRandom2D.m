function dataset = generateRandom2D(dim,start_pts, goal_pts)


cols=dim(1);
rows=dim(2);
dataset.cols = cols;
dataset.rows = rows;
dataset.cell_size = 0.01;
dataset.origin_x = -rows*dataset.cell_size/2;
dataset.origin_y = -cols*dataset.cell_size/2;

dataset.map = zeros(dataset.rows, dataset.cols);

n_rects = 8;
rects = cell(n_rects);
while true
    % Generate obstacles that don't overlap with start and end configurations
    for i = 1:n_rects
        while true
            rects{i} = random_rect(dataset.rows, dataset.cols, 1, 1);
            if (check_arm_collision(start_pts, rects{i}, dataset) ...
                && check_arm_collision(goal_pts, rects{i}, dataset)), break; end
        end
    end
    
    % Check that obstacles don't overlap
    good_rect = true;
    for i = 1:n_rects
        for j = (i+1):n_rects
            good_rect = good_rect && ...
                check_bad_rect(dataset.rows, dataset.cols, rects{i}, rects{j});
        end
    end
    if good_rect, break; end
end

for i = 1:n_rects
    rect = rects{i};
    dataset.map(rect(1):rect(2), rect(3):rect(4)) = 1;
end

end

function rect = random_rect(rows, cols, start_x, start_y)

% Returns [x_bot, x_top, y_left, y_right]

height = randi([10, 50]);
width = randi([10, 50]);
% height = randi([20, floor(rows/3)]);
% width = randi([20, floor(cols/3)]);
x_bot = randi([start_x, rows-height]);
y_left = randi([start_y, cols-width]);

rect = [x_bot, x_bot+height, y_left, y_left+width];
end

function ok = check_goal_collision(goal, rect, dataset)

goal_img_frame = [(-dataset.origin_x + goal(1))/dataset.cell_size,...
                  (-dataset.origin_y + goal(2))/dataset.cell_size];
              
inside_rect = (goal_img_frame(2) >= rect(1) && goal_img_frame(2) <= rect(2) ...
            && goal_img_frame(1) >= rect(3) && goal_img_frame(1) <= rect(4));

ok = ~inside_rect;

end

function ok = check_arm_collision(pts, rect, dataset)

% Add origin to set of points
pts=cat(2,[dataset.origin_x,dataset.origin_y,0]',pts);

for i=2:size(pts,2)
    
    % Arm-link markers
    markers=[linspace(pts(1,i-1),pts(1,i),5);...
             linspace(pts(2,i-1),pts(2,i),5)];
         
    % Check markers for collision
    for j=1:size(markers,2)
        m_img_frame = ceil([(-dataset.origin_x + markers(1,j))/dataset.cell_size,...
                             (-dataset.origin_y + markers(2,j))/dataset.cell_size]);

        inside_rect = (m_img_frame(2) >= rect(1) && m_img_frame(2) <= rect(2) ...
                    && m_img_frame(1) >= rect(3) && m_img_frame(1) <= rect(4));
        if inside_rect
            break
        end
    end
    
    if inside_rect
        break
    end
end

ok=~inside_rect;

end

function ok = check_origin_collision(rect, dataset)

origin_img_frame = [(-dataset.origin_x)/dataset.cell_size,...
                    (-dataset.origin_y)/dataset.cell_size];
              
inside_rect = (origin_img_frame(2) >= rect(1) && origin_img_frame(2) <= rect(2) ...
            && origin_img_frame(1) >= rect(3) && origin_img_frame(1) <= rect(4));

ok = ~inside_rect;

end

function ok = check_bad_rect(rows,cols,rect1,rect2)
    
a1=zeros(rows,cols);
a1(rect1(1):rect1(2), rect1(3):rect1(4)) = 1;

a2=zeros(rows,cols);
a2(rect2(1):rect2(2), rect2(3):rect2(4)) = 1;

a=a1+a2;

ok = max(a(:))<=1 ;
    
end

