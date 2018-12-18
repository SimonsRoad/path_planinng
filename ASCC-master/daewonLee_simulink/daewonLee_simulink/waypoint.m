function output = waypoint(input)

% output = [sin(input/2),cos(input/2),0,0];
    if input>48
        output = [0,0,0,0];
    elseif input>43
        output = [0,0,5,0];    
    elseif input>36
        output = [0,5,5,0];
    elseif input>17
        output = [5,5,5,0];
    elseif input>15
        output = [5,0,5,0];
    else
    output = [0,0,5,0];
    end
end
