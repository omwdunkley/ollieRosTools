function matlab2opencv( variable, fileName, flag)

[rows cols] = size(variable);

% Beware of Matlab's linear indexing
variable = variable';

% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w'; 
end

if ( ~exist(fileName,'file') || flag == 'w' )
    % New file or write mode specified 
    file = fopen( fileName, 'w');
    fprintf( file, '%%YAML:1.0\n');
else
    % Append mode
    file = fopen( fileName, 'a');
end

% Write variable header
fprintf( file, '    %s: !!opencv-matrix\n', inputname(1));
fprintf( file, '        rows: %d\n', rows);
fprintf( file, '        cols: %d\n', cols);
fprintf( file, '        dt: f\n');
fprintf( file, '        data: [ ');

% Write variable data
for i=1:rows*cols
    fprintf( file, '%.6f', variable(i));
    if (i == rows*cols), break, end
    fprintf( file, ', ');
    if mod(i+1,4) == 0
        fprintf( file, '\n            ');
    end
end

fprintf( file, ']\n');

fclose(file);

%matlab2opencv(features_img(features_img(:,3,33)==1,:,33), '/home/ollie/test/eigen/data/f2d_33.yml')
% 
% #include <iostream>
% #include <string>
% 
% #include <cv.h>
% #include <highgui.h>
% 
% using namespace cv;
% using namespace std;
% 
% int main (int argc, char * const argv[])
% {   
%     Mat var1;
%     Mat var2;
% 
%     string demoFile  = "demo.yml";
% 
%     FileStorage fsDemo( demoFile, FileStorage::READ);
%     fsDemo["Variable1"] >> var1;
%     fsDemo["Variable2"] >> var2;
% 
%     cout << "Print the contents of var1:" << endl;
%     cout << var1 << endl << endl;
% 
%     cout << "Print the contents of var2:" << endl;
%     cout << var2 << endl;
% 
%     fsDemo.release();
%     return 0;
% }