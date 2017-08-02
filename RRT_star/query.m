        function p_ = query(rrt, xstart, xgoal)
        %RRT.query Find a path between two points
        %
        % X = R.path(START, GOAL) finds a path (Nx3) from pose START (1x3) 
        % to pose GOAL (1x3).  The pose is expressed as [X,Y,THETA]. 
        %
        % R.path(START, GOAL) as above but plots the path in 3D, where the vertical
        % axis is vehicle heading angle.  The nodes are shown as circles and the
        % line segments are blue for forward motion and red for backward motion.
        %
        % Notes::
        % - The path starts at the vertex closest to the START state, and ends
        %   at the vertex closest to the GOAL state.  If the tree is sparse this
        %   might be a poor approximation to the desired start and end.
        %
        % See also RRT.plot.

            assert(rrt.graph.n > 0, 'RTB:RRT: there is no plan');
            rrt.checkquery(xstart, xgoal);
            
            g = rrt.graph;
            vstart = g.closest(xstart);
            vgoal = g.closest(xgoal);

            % find path through the graph using A* search
            [path,cost] = g.Astar(vstart, vgoal);
            
            fprintf('A* path cost %g\n', cost);
          
            % concatenate the vehicle motion segments
            cpath = [];
            for i = 1:length(path)
                p = path(i);
                data = g.vdata(p);
                if ~isempty(data)
                    if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                        cpath = [cpath data.path];
                    else
                        cpath = [cpath data.path(:,end:-1:1)];
                    end
                end
            end

            if nargout == 0
                % plot the path
                clf; hold on

                plot2(g.coord(path)', 'o');     % plot the node coordinates
                
                for i = 1:length(path)
                    p = path(i);
                    b = g.vdata(p);            % get path data for segment
                    
                    % draw segment with direction dependent color
                    if ~isempty(b)
                        % if the vertex has a path leading to it
                        
                        if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                            % positive edge
                            %  draw from prev vertex to end of path
                            seg = [g.coord(path(i-1)) b.path]';
                        else
                            % negative edge
                            %  draw reverse path to next next vertex
                            seg = [  b.path(:,end:-1:1)  g.coord(path(i+1))]';
                        end
                        
                        if b.vel > 0
                            plot2(seg, 'b');
                        else
                            plot2(seg, 'r');
                        end
                    end
                end

                xlabel('x'); ylabel('y'); zlabel('\theta');
                grid
            else
                p_ = cpath';
            end
        end
