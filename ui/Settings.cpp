/*!

 Settings.h
 CS123 Support Code

 @author  Evan Wallace (edwallac)
 @date    9/1/2010

 This file contains various settings and enumerations that you will need to
 use in the various assignments. The settings are bound to the GUI via static
 data bindings.

**/

#include "Settings.h"
#include <QFile>
#include <QSettings>

Settings settings;


/**
 * Loads the application settings, or, if no saved settings are available, loads default values for
 * the settings. You can change the defaults here.
 */
void Settings::loadSettingsOrDefaults() {
    // Set the default values below
    QSettings s("CS123", "CS123");

    // Brush
    transformationType = s.value("transformationType", TRANSFORMATION_BRDF).toInt();
    diffuseColor.r = s.value("diffuseRed", 255).toInt();
    diffuseColor.g = s.value("diffuseGreen", 255).toInt();
    diffuseColor.b = s.value("diffuseBlue", 255).toInt();
    diffuseColor.a = s.value("diffuseAlpha", 1).toInt();

    specularColor.r = s.value("specularRed", 127).toInt();
    specularColor.g = s.value("specularGreen", 127).toInt();
    specularColor.b = s.value("specularBlue", 127).toInt();
    specularColor.a = s.value("specularAlpha", 1).toInt();

    smoothing = s.value("smoothing", 0.4f).toDouble();
    curvature = s.value("curvature", 1).toInt();

    sValue = s.value("sValue", 50).toInt();
    frosty = s.value("frosty", 1).toInt();
    darkness = s.value("darkness", 0).toFloat();
    ht = s.value("ht", 0).toDouble();

//    imagePath = s.value("imagePath", "images/han.jpg");
    imagePath = s.value("imagePath", "images/han.jpg").toString();
    backgroundPath = s.value("backgroundPath", "images/background.jpg").toString();
    texturePath = s.value("texturePath", "images/marble.jpg").toString();


    // Shapes
//    useLighting = s.value("useLighting", true).toBool();


    currentTab = s.value("currentTab", TAB_2D).toBool();
}

void Settings::saveSettings() {
    QSettings s("CS123", "CS123");

    // Brush
    s.setValue("transformationType", transformationType);
    s.setValue("diffuseRed", diffuseColor.r);
    s.setValue("diffuseGreen", diffuseColor.g);
    s.setValue("diffuseBlue", diffuseColor.b);
    s.setValue("diffuseAlpha", diffuseColor.a);

    s.setValue("specularRed", specularColor.r);
    s.setValue("specularGreen", specularColor.g);
    s.setValue("specularBlue", specularColor.b);
    s.setValue("specularAlpha", specularColor.a);

    s.setValue("smoothing", smoothing);
    s.setValue("curvature", curvature);

    s.setValue("sValue", sValue);
    s.setValue("frosty", frosty);
    s.setValue("darkness", darkness);
    s.setValue("ht", ht);

    s.setValue("imageComboValue", imagePath);
    s.setValue("backgroundComboValue", backgroundPath);
    s.setValue("textureComboValue", texturePath);

    // Shapes
//    s.setValue("useLighting", useLighting);
//    s.setValue("drawWireframe", drawWireframe);
//    s.setValue("drawNormals", drawNormals);

    s.setValue("currentTab", currentTab);
}

int Settings::getSceneMode() {
//    if (this->useSceneviewScene)
//        return SCENEMODE_SCENEVIEW;
//    else
//        return SCENEMODE_SHAPES;
}
