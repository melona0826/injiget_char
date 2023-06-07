#include <cstdlib>
#include <string>
#include <iostream>

using namespace std;

int main()
{
  string pkg_name = "injiget_char ";
  string launch_name_1 = "pallet_pick.launch ";
  string launch_name_2 = "line_tracking.launch ";
  string launch_name_3 = "ocr.launch";
  string launch_cmd = "roslaunch ";
  string cmd = launch_cmd + pkg_name + launch_name_1;

  int result = system(cmd.c_str());

  cmd = launch_cmd + pkg_name + launch_name_2;
  result = system(cmd.c_str());

  cmd = launch_cmd + pkg_name + launch_name_3;
  result = system(cmd.c_str());


}
