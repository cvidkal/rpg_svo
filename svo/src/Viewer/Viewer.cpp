//
// Created by fg on 08/07/2017.
//

#include "Viewer.h"
#include <thread>
#include <Eigen/Eigen>


Viewer::Viewer(int width,int height){

    mbRunning = true;
    pangolin::CreateWindowAndBind("Main",width,height);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    cameraMatrix_ =  pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(width,height,420,420,width/2,height/2,0.2,100),
            pangolin::ModelViewLookAt(0,0,10, 0,0,-4, pangolin::AxisY)
    );
}

Viewer::~Viewer(){

}

void Viewer::run(){

    mainLoop();
    std::thread* viewer = new std::thread(&Viewer::mainLoop,this);
    viewer->join();

}


void Viewer::mainLoop(){



    Eigen::Matrix3d K;
    K<<480,0,320,0,480,240,0,0,1;
    Eigen::Matrix3d Kinv = K.inverse();
    Eigen::Vector3d mPosition(0,0,5);
//    Eigen::Quaterniond q(0.240749,-0.76113,-0.355916,-0.485843);
    Eigen::Quaterniond q(1,0,0,0);
    Eigen::Matrix3d mRgv;
    mRgv<<1,0,0,0,-1,0,0,0,-1;
    Eigen::Matrix4d T;
    T<<mRgv*q.toRotationMatrix(),mRgv*mPosition,Eigen::Vector4d(0,0,0,1).transpose();



    //UI
    const int UI_WIDTH = 180;
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::Handler3D handler(cameraMatrix_);
    pangolin::View& space3 = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
            .SetHandler(&handler);


    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui")
            .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));


    pangolin::Var<bool> ViewMode("ui.ViewMode",false,false);
    pangolin::Var<bool> VideoMode("ui.VideoMode",false,false);

    //KeyBoard callback
    {
        pangolin::RegisterKeyPressCallback('r', pangolin::ToggleVarFunctor("ui.ViewMode"));
        pangolin::RegisterKeyPressCallback('v', pangolin::ToggleVarFunctor("ui.VideoMode"));

    }


    while(!pangolin::ShouldQuit()){

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        //-----------------3D Mode------------------
        if(ViewMode.Get()){

        space3.Activate(cameraMatrix_);//set ViewMatrix

            // Draw Objects
            if(VideoMode.Get()){
                static int internal_id_imu = 0;
                static int internal_id_vision = 0;
                if(internal_id_imu<estimates_imu_.size()&&internal_id_vision<estimate_vision_.size()){
                    double ts_imu = estimates_imu_[internal_id_imu].first;
                    double ts_vision = estimate_vision_[internal_id_vision].first;
                    if(ts_imu<ts_vision){
                        pangolin::glDrawAxis<double>(estimates_imu_[internal_id_imu].second,2.0);
                        internal_id_imu++;
                    }
                    else{

                    }
                }

            }

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();
//        pangolin::glDrawFrustrum<double>(Kinv,640,480,T,5.);
        pangolin::glDrawAxis<double>(T,2.);
        // Swap frames and Process Events
        }
            //------------------2D Mode------------------
        else{

        }



        pangolin::FinishFrame();

    }


}

void Viewer::pushIMUEstimate(double timestamp, Eigen::Matrix<double, 4, 4, 0, 4, 4> Twi) {

    Twi.row(1) *=-1;
    Twi.row(2) *=-1;
    estimates_imu_.emplace_back(timestamp,Twi);
}


void Viewer::pushVisionEstimate(double timestamp, Eigen::Matrix<double, 4, 4, 0, 4, 4> Twf) {
    Twf.row(1) *=-1;
    Twf.row(2) *=-1;
    estimate_vision_.emplace_back(timestamp,Twf);
}
