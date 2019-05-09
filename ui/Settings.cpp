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
    diffuseColor.r = s.value("diffuseRed", 0).toInt();
    diffuseColor.g = s.value("diffuseGreen", 0).toInt();
    diffuseColor.b = s.value("diffuseBlue", 0).toInt();
    diffuseColor.a = s.value("diffuseAlpha", 1).toInt();

    specularColor.r = s.value("specularRed", 0).toInt();
    specularColor.g = s.value("specularGreen", 0).toInt();
    specularColor.b = s.value("specularBlue", 0).toInt();
    specularColor.a = s.value("specularAlpha", 1).toInt();

    smoothing = s.value("smoothing", 0).toDouble();
    curvature = s.value("curvature", 0).toInt();

    sValue = s.value("sValue", 0).toDouble();
    frosty = s.value("frosty", 0).toInt();
    darkness = s.value("darkness", 0).toInt();
    ht = s.value("ht", 0).toDouble();


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
