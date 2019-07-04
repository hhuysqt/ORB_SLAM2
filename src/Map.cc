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

#include "Map.h"

#include<mutex>

#include <unistd.h>
#include <sys/stat.h>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::set_path(std::string path)
{
    std::ostringstream dirname;

    /* mkdir /<path>/imgX */
    for (int cnt = 0; ; cnt++) {
        dirname << path << "/img" << cnt;
        struct stat stat1;
        if (stat(dirname.str().c_str(), &stat1) != 0) {
            break;
        }
        dirname.str("");
    }
    mkdir (dirname.str().c_str(), 0777);
    imgpath = dirname.str();
}

void Map::AddKeyFrame(KeyFrame *pKF, cv::Mat &img)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;

    // save the keyframe image
    if (imgpath.length() != 0) {
        std::ostringstream tmp;
        tmp << imgpath << "/img" << pKF->mnId << ".jpg";
        cv::imwrite(tmp.str().c_str(), img);
    }
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint

    // remove the corresponding image
    if (imgpath.length() != 0) {
        std::ostringstream tmp;
        tmp << imgpath << "/img" << pKF->mnId << ".jpg";
        cout << tmp.str() << endl;
        remove(tmp.str().c_str());
    }
}

void Map::save_matching_pairs(void)
{
    std::ostringstream tmp;
    tmp << imgpath << "/matching.txt";
    std::cout << "Saving matching pairs for COLMAP: " << tmp.str() << std::endl;

    ofstream f;
    f.open(tmp.str());

    set<KeyFrame*> already_matched;

    for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(), 
         send = mspKeyFrames.end(); 
         sit != send; sit++) {
        std::ostringstream tmp;
        tmp << "img" << (*sit)->mnId << ".jpg";
        already_matched.insert(*sit);
        set <KeyFrame*> cnned_kf = (*sit)->GetConnectedKeyFrames();
        for (set<KeyFrame*>::iterator c_sit = cnned_kf.begin(),
             c_send = cnned_kf.end();
             c_sit != c_send; c_sit++) {
            if (already_matched.count(*c_sit))
                continue;

            // write it to the matching file
            std::ostringstream tmp2;
            tmp2 << "img" << (*c_sit)->mnId << ".jpg";
            f << tmp.str() << " " << tmp2.str() << endl;
        }
    }

    f.close();
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
