robotology_root = '/home/arturo/Code/advr-superbuild';
robotology_lib_dir = [robotology_root '/build/install/lib'];
robotology_include_dir = [robotology_root '/build/install/include'];
ros_lib_dir = '/opt/ros/kinetic/lib';
ros_include_dir = '/opt/ros/kinetic/include';
eigen3_include_dir = '/usr/include/eigen3';



include_flags = ['-I' robotology_include_dir ' -I' ros_include_dir ' -I' eigen3_include_dir];
linkdir_flags = ['-L' robotology_lib_dir ' -L' ros_lib_dir];
lib_flags = '-lXBotInterface -lkdl_parser -leigen_conversions -lorocos-kdl';

setenv('ROBOTOLOGY_ROOT', robotology_root)
setenv('LD_LIBRARY_PATH', [getenv('LD_LIBRARY_PATH') ':' robotology_lib_dir ':' ros_lib_dir]);
setenv('ROS_MASTER_URI', 'http://localhost:11311')

mex('gcomp_example.cpp', '-v', '-g', ['-I' robotology_include_dir], ['-I' ros_include_dir], ['-I' eigen3_include_dir],  ['-L' robotology_lib_dir], ['-L' ros_lib_dir], '-lXBotInterface', '-lkdl_parser', '-leigen_conversions', '-lorocos-kdl');

% mex gcomp_example.cpp -g 
% -I/home/arturo/Code/advr-superbuild/build/install/include -I/opt/ros/kinetic/include -I/usr/include/eigen3 
% -L"/home/arturo/Code/advr-superbuild/build/install/lib" -L"/opt/ros/kinetic/lib" 
% -lXBotInterface -lkdl_parser -leigen_conversions -lorocos-kdl