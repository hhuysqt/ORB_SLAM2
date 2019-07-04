/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <System.h>

#include <signal.h>
#include <unistd.h>

using namespace std;

ORB_SLAM2::System* SLAM;

string vocabulary_file("");
string setting_file("");
string working_dir(".");
string video_file("");

void on_quit(int s)
{
    SLAM->Shutdown();
    //SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM->save_matching_pair();

    exit(0);
}

void print_usage(void)
{
  cerr << "Usage: mono_video [OPTION] [FILE]" << endl;
  cerr << "-h  help" << endl;
  cerr << "-v  vocabulary file" << endl;
  cerr << "-s  setting file" << endl;
  cerr << "-f  video file" << endl;
  cerr << "-d  working directory to save keyframe image and matching file" << endl;
  exit(-1);
}

int main(int argc, char **argv)
{
  int c;
  while ((c = getopt(argc, argv, "v:s:f:d:")) != -1) {
    switch (c) {
      case 'v':
        vocabulary_file = string(optarg);
        break;
      case 's':
        setting_file = string(optarg);
        break;
      case 'f':
        video_file = string(optarg);
        break;
      case 'd':
        working_dir = string(optarg);
        break;
      default:
        print_usage();
        exit(-1);
    }
  }

  if (video_file.length() < 1 || vocabulary_file.length() < 1 ||
      setting_file.length() < 1) {
    print_usage();
    exit(-1);
  }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(vocabulary_file, setting_file, ORB_SLAM2::System::MONOCULAR,true);
    SLAM->set_path(working_dir);

    cv::VideoCapture cap;
    cap.open(video_file);
    if (!cap.isOpened()) {
      cerr << "Failed to open " << video_file << endl;
      return -1;
    }

    // capture ctrl-c signal
    struct sigaction sig;
    sig.sa_handler = on_quit;
    sigemptyset(&sig.sa_mask);
    sig.sa_flags = 0;
    sigaction(SIGINT, &sig, NULL);

    cv::Mat frame;
    cap >> frame;
    double ts = 0.0;
    while (!frame.empty()) {
      SLAM->TrackMonocular(frame, ts);
      ts = ts + 1;
      cap >> frame;
    }

    on_quit(0);
}

