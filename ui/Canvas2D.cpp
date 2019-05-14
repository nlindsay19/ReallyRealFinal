/**
 * @file Canvas2D.cpp
 *
 * CS123 2-dimensional canvas. Contains support code necessary for Brush, Filter, Intersect, and
 * Ray.
 *
 * YOU WILL NEED TO FILL THIS IN!
 *
 */

// For your convenience, a few headers are included for you.
#include <assert.h>
#include <iostream>
#include <math.h>
#include <memory>
#include <unistd.h>
#include "Canvas2D.h"
#include "Settings.h"
#include "RayScene.h"
#include <QKeyEvent>
#include "materials/brdfreplacement.h"
#include "materials/materialmanager.h"
#include <QPainter>

Canvas2D::Canvas2D() :
    // @TODO: Initialize any pointers in this class here.
    m_rayScene(nullptr)
{
    setFocusPolicy(Qt::StrongFocus);
    m_hDown = 0;
}

Canvas2D::~Canvas2D()
{
}

// This is called when the canvas size is changed. You can change the canvas size by calling
// resize(...). You probably won't need to fill this in, but you can if you want to.
void Canvas2D::notifySizeChanged(int w, int h) {
}

void Canvas2D::paintEvent(QPaintEvent *e) {
    // You probably won't need to fill this in, but you can if you want to override any painting
    // events for the 2D canvas. For now, we simply call the superclass.
    SupportCanvas2D::paintEvent(e);

}

// ********************************************************************************************
// ** BRUSH
// ********************************************************************************************


void Canvas2D::mouseDown(int x, int y) {
    // @TODO: [BRUSH] Mouse interaction for brush. You will probably want to create a separate
    //        class for each of your brushes. Remember that you can use the static Settings
    //        object to get the currently selected brush and its parameters.

    // You're going to need to leave the alpha value on the canvas itself at 255, but you will
    // need to use the actual alpha value to compute the new color of the pixel

//    bool fixAlphaBlending = settings.fixAlphaBlending; // for extra/half credit

}

void Canvas2D::mouseDragged(int x, int y) {
    // TODO: [BRUSH] Mouse interaction for Brush.

    if(settings.transformationType == TRANSFORMATION_LIGHTING){
        if(m_drawnColors.size() == 0){
            for(int i = 0; i < m_image->height(); i++){
                for(int j = 0; j < m_image->width(); j++){
                    m_drawnColors.push_back(Eigen::Vector3f(0,0,0));
                    m_highlightColors.push_back(Eigen::Vector3f(0,0,0));
                }
            }
        }
        if(y < m_image->height() && x < m_image->width() && y > 0 && x > 0){
            int index = y * m_image->width() + x;
            m_drawnColors[index] = Eigen::Vector3f((float)settings.diffuseColor.r, (float)settings.diffuseColor.g, (float)settings.diffuseColor.b);
            m_image->setPixelColor(x, y, QColor(settings.diffuseColor.r, settings.diffuseColor.g, settings.diffuseColor.b));
        }
        if(m_hDown){
            if(y < m_image->height() && x < m_image->width() && y > 0 && x > 0){
                int index = y * m_image->width() + x;
                m_highlightColors[index] = Eigen::Vector3f((float)settings.specularColor.r, (float)settings.specularColor.g, (float)settings.specularColor.b);
                m_image->setPixelColor(x, y, QColor(settings.specularColor.r, settings.specularColor.g, settings.specularColor.b));
                highlight = Eigen::Vector2f(x,y);
                if(envmapRows > 0){
                    BrdfReplacement br;
                    br.specularDirs = envmapSpecularDirs;
                    std::vector<Eigen::Vector3f> copyImage = envmapImage;
                    br.addHighlightsToEnvmap(envmapImage, envmapMask, envmapNormals, envmapRows, envmapCols, m_highlightColors, highlight, settings.sValue * 2.0f);
                    MaterialManager mm;
                    mm.vectorToFile(envmapImage, "images/output.png", envmapRows, envmapCols);
                    envmapImage = copyImage;
                    if (!loadImage("images/output.png")) {
                        std::cout << "couldn't load image" << std::endl;
                    }

                }
            }
        }
    }
    else if(settings.transformationType == TRANSFORMATION_GLASS){
        if(m_highlightColors.size() == 0){
            for(int i = 0; i < m_image->height(); i++){
                for(int j = 0; j < m_image->width(); j++){
                    m_highlightColors.push_back(Eigen::Vector3f(0,0,0));
                }
            }
        }
        if(m_hDown){
            if(y < m_image->height() && x < m_image->width() && y > 0 && x > 0){
                int index = y * m_image->width() + x;
                m_highlightColors[index] = Eigen::Vector3f((float)settings.specularColor.r, (float)settings.specularColor.g, (float)settings.specularColor.b);
                m_image->setPixelColor(x, y, QColor(settings.specularColor.r, settings.specularColor.g, settings.specularColor.b));
                highlight = Eigen::Vector2f(x,y);
                if(envmapRows > 0){
                    BrdfReplacement br;
                    br.specularDirs = envmapSpecularDirs;
                    std::vector<Eigen::Vector3f> copyImage = envmapImage;

                    br.addHighlightsToEnvmap(envmapImage, envmapMask, envmapNormals, envmapRows, envmapCols, m_highlightColors, highlight, settings.sValue * 2.0f);
                    MaterialManager mm;
                    mm.vectorToFile(envmapImage, "images/output.png", envmapRows, envmapCols);
                    envmapImage = copyImage;
                    if (!loadImage("images/output.png")) {
                        std::cout << "couldn't load image" << std::endl;
                    }

                }
            }
        }
    }
    update();

}

void Canvas2D::mouseUp(int x, int y) {
    // TODO: [BRUSH] Mouse interaction for Brush.


}



// ********************************************************************************************
// ** FILTER
// ********************************************************************************************

void Canvas2D::filterImage() {
//    // TODO: [FILTER] Filter the image. Some example code to get the filter type is provided below.

//    switch(settings.filterType) {
//        case FILTER_BLUR:
//            // ...
//            break;
//            // fill in the rest
//    }

}

void Canvas2D::setScene(RayScene *scene) {
    m_rayScene.reset(scene);
}

void Canvas2D::renderImage(Camera *camera, int width, int height) {
    if (m_rayScene) {
        // @TODO: raytrace the scene based on settings
        //        YOU MUST FILL THIS IN FOR INTERSECT/RAY

        // If you want the interface to stay responsive, make sure to call
        // QCoreApplication::processEvents() periodically during the rendering.

    }
}

void Canvas2D::cancelRender() {
    // TODO: cancel the raytracer (optional)
}



void Canvas2D::settingsChanged() {
    // TODO: Process changes to the application settings.
}

void Canvas2D::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_H) {
        m_hDown = 1;
    }
}

void Canvas2D::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_H) {
        m_hDown = 0;
    }
}
