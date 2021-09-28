function progress(N)
% PROGRESS Just shows a text progress bar. 
%
%   utils.progress(N)
%
% N should be a number from 0-1 to indicate progress.
% Note: you MUST call utils.progress(0) at the start of the loop to
% initialize the display properly.
%
% Alternatively, if N is a text argument, then this is equivalent to
% calling the function with N=0 (initialize), but will also display the
% task being done (from the text argument)

persistent ticStart

if isstring(N) || ischar(N)
    fprintf("%s", N);
    N = 0;
end
percent = round(N * 100);
w = 40; % Width of progress bar

if ~isempty(ticStart)
    t = seconds(toc(ticStart));
    rem = -t+t/N;
    [th, tm, ts] = hms(t);
    [remh, remm, rems] = hms(rem);
else 
    th = 0;
    tm = 0;
    ts = 0;
    remh = 0;
    remm = 0;
    rems = 0;
end

if N == 0
    % Draw empty line
    disp([newline, '  0%[', repmat(' ', 1, w+1), ']', ...
          newline, repmat(' ', 1, 39)]);  
    ticStart = tic;

elseif N >= 1
    % Draw complete line
    disp([repmat(char(8), 1, (w+9+40)), newline, '100%[', repmat('=', 1, w+1), ']', ...
          newline, sprintf('Done. Took %02d:%02d:%02.0f.', th, tm, ts)]);
        
else
    % Draw somewhere in the middle
    perc = sprintf('%3.0f%%', percent); % 4 characters wide, percentage
    disp([repmat(char(8), 1, (w+9+40)), newline, perc, '[', repmat('=', 1, round(percent*w/100)), '>', repmat(' ', 1, w - round(percent*w/100)), ']', ...
          newline, sprintf('(%02d:%02d:%02.0f ellapsed, %02d:%02d:%02.0f remaining)', ...
            th, tm, ts, remh, remm, rems)]);

end
