function [target_path_x, target_path_y,tracker] = set_target_tracker()
set(gcf, 'Name', 'makemap');
fprintf('  t - target path selection \n');
fprintf('  p - tracker start point \n');
fprintf('  e - erase target path\n');
fprintf('  q - leave editing mode\n');

target_path_x = [];
target_path_y = [];
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
                 target_path_x = [target_path_x  point(1,1)];
                 target_path_y = [target_path_y  point(1,2)];
                 hold on
                 h = plot(point(1,1), point(1,2), 'rx');
                 drawnow

                if k1 ==1                     
                    c1 = get(gcf, 'CurrentCharacter');
                    if c1 == 'w'
                        title('')
                        target_path_x(end) = []; target_path_y(end)= [];
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
            target_path_x = [];
            target_path_y = [];
            fprintf('erased target_path');            
            otherwise
             break;      % key pressed
        end
           
     end
end
