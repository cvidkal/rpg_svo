//
// Created by fg on 08/07/2017.
//

#include <slamsystem.h>
#include <Viewer/Viewer.h>

using namespace slam;

int main(){

//    SlamSystem system;

    Viewer * viewer = new Viewer(640,480);
    viewer->run();

}