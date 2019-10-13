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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<double> vTimesTrack;
    vTimesTrack.resize(nImages);

    vector<double> vTimesRelocalization;
    vector<double> vTimesTrackLocalMap;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        cout << "new frame inserted..." << endl;

        // Read image from file
        im = cv::imread(string(argv[3]) + "/" + vstrImageFilenames[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if(ni < nImages-1)
            T = vTimestamps[ni+1] - tframe;
        else if(ni > 0)
            T = tframe - vTimestamps[ni-1];

        if(ttrack < T)
            usleep((T - ttrack) * 1e6);

        cout << "frame tracking done..." << endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    SLAM.GetTrackLocalMapTimes(vTimesTrackLocalMap);
    if(int vTimeSize = vTimesTrackLocalMap.size())
    {
        sort(vTimesTrackLocalMap.begin(), vTimesTrackLocalMap.end());
        double totaltime = 0;
        for (int ni = 0; ni < vTimeSize; ni++)
        {
            totaltime += vTimesTrackLocalMap[ni];
        }
        cout << endl << "-------" << endl << endl;
        cout << "max tracking local map time: " << vTimesTrackLocalMap[vTimeSize-1] << endl;
        cout << "median tracking local map time: " << vTimesTrackLocalMap[vTimeSize/2] << endl;
        cout << "min tracking local map time: " << vTimesTrackLocalMap[0] << endl;
        cout << "mean tracking local map time: " << totaltime / vTimeSize << endl;        
    }

    SLAM.GetRelocalizationTimes(vTimesRelocalization);
    if(int vTimeSize = vTimesRelocalization.size())
    {
        sort(vTimesRelocalization.begin(), vTimesRelocalization.end());
        double totaltime = 0;
        for (int ni = 0; ni < vTimeSize; ni++)
        {
            totaltime += vTimesRelocalization[ni];
            // cout << "vTimesRelocalization[" << ni << "] = " << vTimesRelocalization[ni] << endl;
        }
        cout << endl << "-------" << endl << endl;
        cout << "max relocalization time: " << vTimesRelocalization[vTimeSize-1] << endl;
        cout << "median relocalization time: " << vTimesRelocalization[vTimeSize/2] << endl;
        cout << "min relocalization time: " << vTimesRelocalization[0] << endl;
        cout << "mean relocalization time: " << totaltime / vTimeSize << endl;        
    }

    {   // Tracking time statistics
        sort(vTimesTrack.begin(), vTimesTrack.end());
        double totaltime = 0;
        for(int ni = 0; ni < nImages; ni++)
        {
            totaltime += vTimesTrack[ni];
        }
        cout << endl << "-------" << endl << endl;
        cout << "max tracking time: " << vTimesTrack[nImages-1] << endl;
        cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
        cout << "min tracking time; " << vTimesTrack[0] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;
    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f, s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
