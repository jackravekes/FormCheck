#include <iostream>
#include <sstream>

#include "NtKinect.h"

using namespace std;

void doJob() {
  NtKinect kinect;
  while (1) {
    kinect.setDepth(true);
    kinect.setSkeleton();
    for (auto person : kinect.skeleton) {
      for (auto joint : person) {
        if (joint.TrackingState == TrackingState_NotTracked) continue;
        DepthSpacePoint dp;
        kinect.coordinateMapper->MapCameraPointToDepthSpace(joint.Position,&dp);
        cv::rectangle(kinect.depthImage, cv::Rect((int)dp.X-5, (int)dp.Y-5,10,10), cv::Scalar(0,0,255),2);
      }
    }
    cv::imshow("depth", kinect.depthImage);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }
  cv::destroyAllWindows();
}

int main(int argc, char** argv) {
  try {
    doJob();
  } catch (exception &ex) {
    cout << ex.what() << endl;
    string s;
    cin >> s;
  }
  return 0;
}
