classdef RC_PlatEMO < PROBLEM
%RC_PLATEMO  PlatEMO problem class of the distribution-network model built by
%RC_setup (decision vector, MP-Opt-Model costs and constraints).
%
%   P = RC_setup(24, opts);
%   [Dec, Obj, Con] = platemo('problem', {@RC_PlatEMO, P}, ...
%                             'algorithm', @NSGAII, 'N', 100, 'maxFE', 30000);
%
%   Requires PlatEMO 4.x (mixed encoding: 1 real, 2 integer, 4 binary).
%   M = 1 for the weighted single objective (opts.m = 1), otherwise the
%   number of objectives. CalDec repairs every new solution with RC_repair
%   (bounds, integrality, DR energy neutrality and budget, BSS thresholds).
    properties
        Data            % problem data from RC_setup
    end
    methods
        function Setting(obj)
            Pd = obj.ParameterSet(struct());
            if ~isfield(Pd, 'D')
                error('RC_PlatEMO: pass the struct from RC_setup, e.g. {@RC_PlatEMO, P}.');
            end
            obj.Data     = Pd;
            obj.M        = Pd.nObj;
            obj.D        = Pd.D;
            obj.lower    = Pd.lower;
            obj.upper    = Pd.upper;
            obj.encoding = Pd.enc;
        end
        function PopDec = CalDec(obj, PopDec)
            PopDec = RC_repair(PopDec, obj.Data);
        end
        function PopObj = CalObj(obj, PopDec)
            PopObj = zeros(size(PopDec, 1), obj.M);
            for i = 1:size(PopDec, 1)
                PopObj(i, :) = RC_fitness(PopDec(i, :), obj.Data);
            end
        end
        function PopCon = CalCon(obj, PopDec)
            PopCon = zeros(size(PopDec, 1), obj.Data.nCon);
            for i = 1:size(PopDec, 1)               % objectives come from the cache
                [~, PopCon(i, :)] = RC_fitness(PopDec(i, :), obj.Data);
            end
        end
    end
end
