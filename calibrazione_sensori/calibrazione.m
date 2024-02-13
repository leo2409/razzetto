clc;
clear;
close all;

%% IMPORT DATA ACCELEROMETER
opts = delimitedTextImportOptions("NumVariables", 3);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["VarName1", "VarName2", "VarName3"];
opts.VariableTypes = ["double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
XUP_t = readtable("./accelerometro/XUP.csv", opts);
YUP_t = readtable("./accelerometro/YUP.csv", opts);
ZUP_t = readtable("./accelerometro/ZUP.csv", opts);
XDOWN_t = readtable("./accelerometro/XDOWN.csv", opts);
YDOWN_t = readtable("./accelerometro/YDOWN.csv", opts);
ZDOWN_t = readtable("./accelerometro/ZDOWN.csv", opts);
XUP = table2array(XDOWN_t);
YUP = table2array(YDOWN_t);
ZUP = table2array(ZDOWN_t);
XDOWN = table2array(XUP_t);
YDOWN = table2array(YUP_t);
ZDOWN = table2array(ZUP_t);


% Clear temporary variables
clear opts

%% CALIBRATION ACCELEROMETER

[A, b] = accelcal(XUP,XDOWN,YUP,YDOWN,ZUP,ZDOWN);