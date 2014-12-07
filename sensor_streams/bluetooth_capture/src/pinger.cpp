#include "ros/ros.h"
#include "bluetooth_capture/PingResult.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream> 


#define ADDR_LEN 17

using namespace std;

int main(int argc, char **argv) {

  ros::init(argc, argv, "bluetooth_pinger");
	
  ros::NodeHandle n;

  ros::Publisher pinger_pub = n.advertise<bluetooth_capture::PingResult>("ping_result", 1000);

  char path[100];
  char path2[1035];
  FILE *fp;
  FILE *ping;
  char *addrFlag;
  char *addrFlag2;
  int ret;
	int name_len;

	string dev_name;
	string mac_addr;
	bool is_present = true;
	bluetooth_capture::PingResult ping_result;

  fp = fopen("file.txt","r"); // read mode

  if( fp == NULL ) {
    ROS_ERROR("bluetooth_capture: Error while opening the file.\n");
    exit(EXIT_FAILURE);
  }

	// Read one mac friendly pair from the file
  // Go through each line in the file
  while (fgets(path, sizeof(path)-1, fp) != NULL) {
  
		// If find "00:" in the line
		// which corresponds to the mac address line
    if (addrFlag = strstr(path,"00:")) {
    
      printf("**************************************************\n");	
      
			// Extract MAC address
			mac_addr.assign(addrFlag, ADDR_LEN);
			cout << "known address mAddr " << mac_addr << endl;
			
		} else if (addrFlag = strstr(path, "Name: ")) {
			addrFlag2 = strstr(path, "\n");
			name_len = addrFlag2 - addrFlag - 6;
			dev_name.assign(&addrFlag[6], name_len);
			cout << "known name devName " << dev_name << endl;
			break;
		}
	}
	
	fclose(fp);

	// Run ubertooth-rx
	string UAP_str = mac_addr.substr(6, 2);
  string LAP_str = mac_addr.substr(9, 2) + mac_addr.substr(12, 2) + mac_addr.substr(15, 2);
	
	string command;
	command = "sudo ./ubertooth-rx -l " + LAP_str + " -u " + UAP_str;
	cout << "command: " << command << endl;
  ping = popen(command.c_str(), "r");
  //setbuf(ping, 0);
  
	// Keep running
	char *rssi_char;
	int rssi;
	string rssi_str;
	int count = 0;
	int test;
  // while (ros::ok() && fgets(path, sizeof(path)-1, ping) != NULL) {
  
  //cout << "size of path -1: " << sizeof(path)-1 << endl;	
  
   
  while ((fgets(path, sizeof(path)-1, ping) != NULL) ) {
  //while ( (test = fgetc(ping)) != EOF) {
   //while ((fgets(ping) != NULL) ) {
		// Go through each line in the file
		// The line we are interested in?
 		if ( !strncmp(path, "systime", 7) ) {
			cout << path << endl;
 			rssi_char = strstr(path, " s=");
 			rssi_str.assign(rssi_char+3, 3);
//			rssi = strtol(rssi_char+2, rssi_char+5, 10);
			rssi = atoi(rssi_str.c_str());
//			cout << "rssi_str: " << rssi_str << " rssi: " << rssi << endl;

			ping_result.header.stamp = ros::Time::now();
			ping_result.mac_addr = mac_addr;
			ping_result.dev_name = dev_name;
			ping_result.is_present = is_present;
			ping_result.rssi = rssi;
			pinger_pub.publish(ping_result);
		}

	
  }



	// Close the command
  fclose(ping);

  return 0;
}

