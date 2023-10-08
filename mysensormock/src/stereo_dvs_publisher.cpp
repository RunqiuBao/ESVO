#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Time.h"
#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "rosgraph_msgs/Clock.h"
#include "ros/console.h"

#include <chrono>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <vector>
#include <thread>

struct Event {
    // Member variables go here
    size_t x;
    size_t y;
    bool polarity;
    float ts;
    // ...
};

std::vector<std::string> SplitString(const std::string& input, const std::string delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(input);

    while (std::getline(tokenStream, token, delimiter[0])) {
        tokens.push_back(token);
    }
    std::string target = "\n";
    size_t pos = tokens[tokens.size() - 1].find(target);
    if (pos != std::string::npos) {
        // Remove the substring using erase
        tokens[tokens.size() - 1].erase(pos, target.length()); // remove \n
    }

    return tokens;
}

std::vector<std::string> SplitTimeString(const std::string& timeString) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(timeString);

    while (std::getline(tokenStream, token, '.')) {
    	tokens.push_back(token);
    }
    return tokens;
}

void LoadEventsIntoMemory(std::vector<Event>& vEvents, std::ifstream& fileLeftData, const bool isBinary=false, long datasize = 0) {
    if (isBinary){
        ROS_INFO("start loading");
        long sizeInByte = datasize * sizeof(Event);
        ROS_INFO_STREAM("sizeof(Event): " << std::to_string(sizeof(Event)));
        std::vector<char> rawData(sizeInByte);
        if (fileLeftData.is_open()) {
            fileLeftData.read(&rawData[0], sizeInByte);
        }
        ROS_INFO_STREAM("sizeInByte: " << std::to_string(sizeInByte) << ", sizeRaw: " << std::to_string(rawData.size()));
        ROS_INFO("start memory copy");
        ROS_INFO_STREAM("max vector size: " << vEvents.max_size());
        ROS_INFO("resize succeeded.");
        for (size_t indexEvent=0; indexEvent < datasize; indexEvent++) {
            Event oneEvent;
            std::memcpy(&oneEvent, &rawData[indexEvent * sizeof(Event)], sizeof(Event));
            vEvents.emplace_back(oneEvent);
        }
    }
    else{
        bool bIsNotEndOfData = true;
        std::string line;
        while (bIsNotEndOfData) {
            std::getline(fileLeftData, line);
            if (fileLeftData.eof()){
                bIsNotEndOfData = false;
            }
            else{
                std::vector<std::string> splitsLine = SplitString(line, " ");  // t, x, y, p
                Event oneEvent;
                oneEvent.x = std::stoi(splitsLine[1]);
                oneEvent.y = std::stoi(splitsLine[2]);
                oneEvent.ts = std::stof(splitsLine[0]);
                oneEvent.polarity = static_cast<bool>(std::stoi(splitsLine[3]));
                vEvents.emplace_back(oneEvent);
            }
        }
        ROS_INFO("finsihed loading one TXT event data file. (%d)", vEvents.size());
    }
}

void SaveToBinary(std::vector<Event>& vEvents, std::string datapath) {
    std::ofstream file;
    file.open(datapath, std::ios::out | std::ios::binary);
    if (file.is_open()) {
        file.write((char*)&vEvents[0], vEvents.size() * sizeof(Event));
    }
    file.close();
}

float GetOneArray(dvs_msgs::EventArray& leftEvents, std::vector<Event>& vEvents, size_t& eventCount, const size_t frameCount, const ros::Time tsBase, const float stackTimeLength, const float tsToStop = 1e9){
    ROS_INFO("Entered GetOneArray");
    leftEvents.height = 720;
    leftEvents.width = 1280;
    std_msgs::Header header;

    std::string line;
    float sLeftStartTimeStamp = -1;
    float fLeftLatestTimeStamp = 0.;
    ros::Time ts;
    while (true) {
        if (eventCount >= vEvents.size()){
            ROS_INFO("end of events.");
            break;
        }
        if (sLeftStartTimeStamp == -1) {
            sLeftStartTimeStamp = vEvents[eventCount].ts;
        }else{
            if (vEvents[eventCount].ts > tsToStop){
                break;
            }
            if ((vEvents[eventCount].ts - sLeftStartTimeStamp) > stackTimeLength) {
                break;
            }
        }
        dvs_msgs::Event oneEvent;
        oneEvent.x = vEvents[eventCount].x;
        oneEvent.y = vEvents[eventCount].y;
        ros::Duration eventTimeBias(vEvents[eventCount].ts - sLeftStartTimeStamp);
        ts = tsBase + eventTimeBias;
        oneEvent.ts = ts;
        oneEvent.polarity = vEvents[eventCount].polarity;
        leftEvents.events.push_back(oneEvent);
        fLeftLatestTimeStamp = vEvents[eventCount].ts;
        eventCount++;
    }
    ROS_DEBUG("finished loading events.");
    if (eventCount < vEvents.size()){
   	    header.stamp = tsBase;
   	    header.seq = frameCount;
   	    header.frame_id = std::to_string(sLeftStartTimeStamp);
   	    leftEvents.header = header; 
    }
    ROS_DEBUG("finished GetOneArray with success.");
    return fLeftLatestTimeStamp;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "mysensormock");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher left_pub = n.advertise<dvs_msgs::EventArray>("/davis/left/events", 100);
  ros::Publisher right_pub = n.advertise<dvs_msgs::EventArray>("/davis/right/events", 100);

  ros::Rate loop_rate(10);
  ROS_INFO("set up publishers.");
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  std::string datarootPath = "/root/data/vibrationRollerNight/seq1/";
  
  // open data files
//   std::filesystem::path leftDataPath = datarootPath;
//   leftDataPath.append("leftcam/DVS_TEXT.txt");
//   std::ifstream fileLeftData(leftDataPath.string());
//   if (!fileLeftData.is_open()) {
//       std::cerr << "Failed to open left data file." << std::endl;
//       return 1; // Exit with an error code
//   }
//   else{
//       std::string line;
//       bool bIsLoop = true;
//       while (bIsLoop) {
//           std::getline(fileLeftData, line);   
// 	  // Process each line here
//           if (line[0] != '#') {
// 	      bIsLoop = false;
// 	  }
//       }
//   }
//   ROS_INFO("left data file opened.");
//   std::filesystem::path rightDataPath = datarootPath;
//   rightDataPath.append("rightcam/DVS_TEXT.txt");
//   std::ifstream fileRightData(rightDataPath.string());
//   if (!fileRightData.is_open()) {
//       std::cerr << "Failed to open right data file." << std::endl;
//       return 1;
//   }
//   else{
//       std::string line;
//       bool bIsLoop = true;
//       while (bIsLoop) {
//           std::getline(fileRightData, line);   
// 	      // Process each line here
//           if (line[0] != '#') {
// 	        bIsLoop = false;
// 	      }
//       }
//   }
//   ROS_INFO("right data file opened.");
//   std::vector<Event> vLeft, vRight;

//   LoadEventsIntoMemory(vLeft, fileLeftData);
//   LoadEventsIntoMemory(vRight, fileRightData);
//   fileLeftData.close();
//   fileRightData.close();

//   std::filesystem::path leftBinPath = datarootPath;
//   leftBinPath.append("leftcam/vLeft.bin");
//   SaveToBinary(vLeft, leftBinPath.string());
//   std::filesystem::path rightBinPath = datarootPath;
//   rightBinPath.append("rightcam/vRight.bin");
//   SaveToBinary(vRight, rightBinPath.string());

  std::vector<Event> vLeft, vRight;
  std::filesystem::path leftDataPath = datarootPath;
  leftDataPath.append("leftcam/vLeft.bin");
  std::ifstream fileLeftData(leftDataPath.string());
  if (!fileLeftData.is_open()) {
      std::cerr << "Failed to open left data file." << std::endl;
      return 1; // Exit with an error code
  }
  LoadEventsIntoMemory(vLeft, fileLeftData, true, 187481619);
  fileLeftData.close();

  std::filesystem::path rightDataPath = datarootPath;
  rightDataPath.append("rightcam/vRight.bin");
  std::ifstream fileRightData(rightDataPath.string());
  if (!fileRightData.is_open()) {
      std::cerr << "Failed to open right data file." << std::endl;
      return 1;
  }
  LoadEventsIntoMemory(vRight, fileRightData, true, 175186432);
  fileRightData.close();

  size_t frameCount = 0;
  size_t eventCountLeft = 0;
  size_t eventCountRight = 0;
  float perStackTimeLength = 0.005;  // sec.

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    auto start_time = std::chrono::high_resolution_clock::now();
    ros::WallTime tsBaseW = ros::WallTime::now();
    ros::Time tsBase;
    tsBase.sec = tsBaseW.sec;
    tsBase.nsec = tsBaseW.nsec;
    dvs_msgs::EventArray leftEvents;
    float leftTimeSpan = GetOneArray(leftEvents, vLeft, eventCountLeft, frameCount, tsBase, perStackTimeLength);
    ROS_INFO_STREAM("frameCount: " << std::to_string(frameCount) << ", leftEvents length: " << std::to_string(leftEvents.events.size()) <<  ", left event timeStamp: " << std::to_string(leftEvents.events[0].ts.sec) <<  " sec, " << std::to_string(leftEvents.events[0].ts.nsec)<< " nsec.");

    dvs_msgs::EventArray rightEvents;
    GetOneArray(rightEvents, vRight, eventCountRight, frameCount, tsBase, perStackTimeLength, leftTimeSpan);
    ROS_INFO_STREAM("frameCount: " << std::to_string(frameCount) << ", rightEvents length: " << std::to_string(rightEvents.events.size()) <<  ", right event timeStamp: " << std::to_string(rightEvents.events[0].ts.sec) <<  " sec, " << std::to_string(rightEvents.events[0].ts.nsec)<< " nsec."); 
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO_STREAM("Loading events caused: " << elapsed_seconds.count() << " sec.");

    left_pub.publish(leftEvents);
    right_pub.publish(rightEvents);

    frameCount++;
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
