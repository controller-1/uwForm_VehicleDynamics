% This helper function imports Pacejka format tire data from a raw .tir
% file. It returns a tire structure containing each parameter found in
% given file.
%
% Modified by Daniel Wageman
% Modified 141015
%
function [tire] = ImportTireData(filename)
%ImportTireData
% ImportTireData( filename ) returns a cell array, C, which contains the
% variable names in the first cell and the data for each variable name.
% in the second cell.

% filename = 'HoosierR25B.tir'      % For testing
% filename = 'tires\HoosierR25A.TIR';

fid = fopen(filename,'rt'); % As of Version 2013b, MATLAB will open files in binary mode on a PC. The string format below relies on the file being opned in text mode which means that 't' has to be appended to the open parameters. Added 10/13 by Daniel Wageman
% fid = fopen(filename,'r'); % As of Version 2013b, MATLAB will open files in binary mode on a PC. The string format below relies on the file being opned in text mode which means that 't' has to be appended to the open parameters. Added 10/13 by Daniel Wageman

C = cell(1,2);
line = 0;

while ~feof(fid)
    line = line + 1;
    [a] = textscan(fid, '%s = %12f','CommentStyle', '$');
    for i = 1:size(a{2},1)
        C{1}(end+1) = a{1}(i);
        C{2}(end+1) = a{2}(i);
    end
end
fclose(fid); % Close file

tire = cell2struct(num2cell(C{2}),C{1},2);

end