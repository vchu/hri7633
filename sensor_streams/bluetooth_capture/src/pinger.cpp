#include "ros/ros.h"
#include "bluetooth_capture/PingResult.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#define ADDR_LEN 17

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bluetooth_pinger");
	
  ros::NodeHandle n;

  ros::Publisher pinger_pub = n.advertise<bluetooth_capture::PingResult>("ping_result", 1000);

  char path[1035];
  char path2[1035];
  FILE *fp;
  FILE *ping;
  char *addrFlag;
  char *addrFlag2;
  int ret;
	int name_len;

	string dev_name;
	string mac_addr;
	bool is_present;
	bluetooth_capture::PingResult ping_result;

  fp = fopen("/home/hri7633/Desktop/file.txt","r"); // read mode

  if( fp == NULL ) {
    ROS_ERROR("bluetooth_capture: Error while opening the file.\n");
    exit(EXIT_FAILURE);
  }

	// Keep running
  while (ros::ok()) {
	// Go through each line in the file
  while (fgets(path, sizeof(path)-1, fp) != NULL) {

		// If find "00:" in the line
		// which corresponds to the mac address line
    if (addrFlag = strstr(path,"00:")) {
    
      printf("**************************************************\n");	
      
			// Extract MAC address
			mac_addr.assign(addrFlag, ADDR_LEN);
			cout << "known address mAddr " << mac_addr << endl;

			// Preset is_present
			is_present = false;

			// Execute hcitool command
      char command[100] = "sudo hcitool info ";
      strncat(command, mac_addr.c_str(), ADDR_LEN);
      printf("pinging for %s\n", command);
      ping = popen(command, "r");

			// Is this MAC address present?
			// Go through each output line of hcitool
      while (fgets(path2, sizeof(path2)-1, ping) != NULL) {
				if (addrFlag2 = strstr(path2,"00:")) {
					is_present = true;
					cout << "found address macAddr2 " << mac_addr << endl;
					break;
				}
      }

			// Close the command
      fclose(ping);

    } else if (addrFlag = strstr(path, "Name: ")) {
			addrFlag2 = strstr(path, "\n");
			name_len = addrFlag2 - addrFlag - 6;
			dev_name.assign(&addrFlag[6], name_len);
			cout << "known name devName " << dev_name << endl;
			
			// Time to publish
			ping_result.header.stamp = ros::Time::now();
			ping_result.mac_addr = mac_addr;
			ping_result.dev_name = dev_name;
			ping_result.is_present = is_present;
			pinger_pub.publish(ping_result);
    } 
  }
  rewind(fp);
	}

  fclose(fp);

  return 0;
}

