function [] = plot_frame( frame,scale, name, width )

narginchk ( 3, 4 ) ;
if nargin < 4; width = 1.5;  end;

%Plots a coordinate system at the given frame

% Get frame origin
[xyzh] = t_translation(frame);
origin = xyzh(1:3);      

% Rotate unit vectors along x,y,z axis
X = origin + scale*frame(1:3, 1);     
Y = origin + scale*frame(1:3, 2);     
Z = origin + scale*frame(1:3, 3);     

% Plot
was_held = ishold;
hold on
plot3([origin(1) X(1)], [origin(2) X(2)], [origin(3) X(3)],'r','LineWidth',width);        
plot3([origin(1) Y(1)], [origin(2) Y(2)], [origin(3) Y(3)],'g','LineWidth',width);    
plot3([origin(1) Z(1)], [origin(2) Z(2)], [origin(3) Z(3)],'b','LineWidth',width); 

if ~was_held
    hold off;
end

if ~strcmp('',name)
    text(origin(1),origin(2),origin(3),name)
end

end

