 function stop = optim_logging_fun(x,optimValues,state)
 % to use this function, in my main file, declare global history 
 % then options = optimoptions(@fmincon,'OutputFcn',@optim_logging_fun);
 global history 
    stop = false; 
     switch state
         case 'init'
             hold on
             history.fval=[];
             history.x=[];
         case 'iter'
         % Concatenate current point and objective function
         % value with history. x must be a row vector.
           history.fval = [history.fval; optimValues.fval];
           history.x = [history.x; reshape(x,1,[])];       
         case 'done'
             hold off
         otherwise
     end
 end