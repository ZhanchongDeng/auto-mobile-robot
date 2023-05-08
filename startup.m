folder_directory = pwd;
folder_names = ["localization" "mapping" "planning" "/iRobotCreateSimulatorToolbox", "utils"];
for name = folder_names
    addpath(append(pwd,'/', name))
end