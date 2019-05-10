/**
 * @file    Settings.h
 *
 * This file contains various settings and enumerations that you will need to use in the various
 * assignments. The settings are bound to the GUI via static data bindings.
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <QObject>
#include "BGRA.h"

// Enumeration values for the Brush types from which the user can choose in the GUI.
enum Transformation_Type {
    TRANSFORMATION_BRDF,
    TRANSFORMATION_RETEXTURE,
    TRANSFORMATION_GLASS,
    TRANSFORMATION_CAUSTIC,
    TRANSFORMATION_GLOSSY,
    TRANSFORMATION_LIGHTING,
    NUM_BRUSH_TYPES
};

// Enumeration values for the two tabs (2D, 3D) at the bottom of the Window.
enum UITab {
    TAB_2D,
    TAB_3D,
    NUM_TABS
};

/**
 * @struct Settings
 *
 * Stores application settings for the CS123 GUI.
 *
 * You can access all app settings through the "settings" global variable.
 * The settings will be automatically updated when things are changed in the
 * GUI (the reverse is not true however: changing the value of a setting does
 * not update the GUI).
*/
struct Settings {
    // Loads settings from disk, or fills in default values if no saved settings exist.
    void loadSettingsOrDefaults();

    // Saves the current settings to disk.
    void saveSettings();

    // Transformations
    int transformationType;      // The user's selected transformation @see TransformationType
    BGRA diffuseColor;
    BGRA specularColor;
    float smoothing;
    int curvature;
    int sValue;
    int frosty;
    float darkness;
    float ht;
    QString imagePath;
    QString backgroundPath;
    QString texturePath;

    // Shapes
//    bool drawWireframe;         // Draw wireframe only
//    bool drawNormals;           // Turn normals on and off

    int getSceneMode();

    int currentTab;

};

// The global Settings object, will be initialized by MainWindow
extern Settings settings;

#endif // SETTINGS_H
