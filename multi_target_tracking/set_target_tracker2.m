function [target1_path_x, target1_path_y,target2_path_x,target2_path_y,tracker] = set_target_tracker2()
set(gcf, 'Name', 'makemap');
fprintf('  t - target1 path selection \n');
fprintf('  r - target2 path selection \n');
fprintf('  p - tracker start point \n');
fprintf('  e - erase target path\n');
fprintf('  q - leave editing mode\n');

target1_path_x = [];
target1_path_y = [];

target2_path_x = [];
target2_path_y = [];


tracker = [];

while 1 
    drawnow
     k = waitforbuttonpress;
     
     if k ==1 % if pressed 
        c = get(gcf, 'CurrentCharacter');
        switch c
          case 't'
            fprintf('click a sequence of points, <w> when done\n')
            title('click a sequence of points, <w> when done')
            
            while 1
                k1 = waitforbuttonpress;                      
                 point = get(gca,'CurrentPoint'); 
                 target1_path_x = [target1_path_x  point(1,1)];
                 target1_path_y = [target1_path_y  point(1,2)];
                 hold on
                 h = plot(point(1,1), point(1,2), 'rx');
                 drawnow

                if k1 ==1                     
                    c1 = get(gcf, 'CurrentCharacter');
                    if c1 == 'w'
                        title('')
                        target1_path_x(end) = []; target1_path_y(end)= [];
                        break
                    end
                end
            end

            case 'r'
            fprintf('click a sequence of points, <w> when done\n')
            title('click a sequence of points, <w> when done')
            
            while 1
                k1 = waitforbuttonpress;                      
                 point = get(gca,'CurrentPoint'); 
                 target2_path_x = [target2_path_x  point(1,1)];
                 target2_path_y = [target2_path_y  point(1,2)];
                 hold on
                 h = plot(point(1,1), point(1,2), 'bx');
                 drawnow

                if k1 ==1                     
                    c1 = get(gcf, 'CurrentCharacter');
                    if c1 == 'w'
                        title('')
                        target2_path_x(end) = []; target2_path_y(end)= [];
                        break
                    end
                end
            end
                        
          case 'p'
            fprintf('click a tracker start point')
            title('click a tracker start point')
            waitforbuttonpress;
            point1 = get(gca,'CurrentPoint');    % button down detected
            hold on
            h = plot(point1(1,1), point1(1,2), 'ko');
            drawnow         
            tracker = [point1(1,1) point1(1,2)]';
            title('')

          case 'e'
            target1_path_x = [];
            target1_path_y = [];
            
            target2_path_x = [];
            target2_path_y = [];
            
            fprintf('erased target_path');            
            otherwise
             break;      % key pressed
        end
           
     end
end
