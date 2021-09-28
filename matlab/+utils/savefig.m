function savefig(f, filename, res, outputType)
    if nargin < 3 || isempty(res)
        res = 300;
    end
    
    if nargin < 4 || isempty(outputType)
        outputType = 'all';
    end
    
    set(f,'PaperType','A4')
    
    resStr = sprintf('-r%d', res);

    if strcmp(outputType, 'png')
        print(f, filename, '-dpng', resStr);
    elseif strcmp(outputType, 'pdf')
        print(f, filename, '-dpdf', resStr);
    elseif strcmp(outputType, 'epsc')
        print(f, filename, '-depsc', resStr);
    else
        print(f, filename, '-dpng', resStr);
        print(f, filename, '-dpdf', resStr);
        print(f, filename, '-depsc', resStr);
    end
    savefig(f, filename);
end