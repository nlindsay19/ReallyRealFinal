/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "Canvas2D.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionNew;
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionQuit;
    QAction *actionRevert;
    QAction *actionCopy3Dto2D;
    QAction *actionClear;
    QAction *actionUseOrbitingCamera;
    QWidget *centralWidget;
    QGridLayout *gridLayout_10;
    QTabWidget *tabWidget;
    QWidget *tab2D;
    QGridLayout *gridLayout_11;
    QScrollArea *scrollArea;
    Canvas2D *canvas2D;
    QWidget *tab3D;
    QGridLayout *gridLayout_12;
    QWidget *canvas3D;
    QDockWidget *transformationDock;
    QWidget *transformationDockContents;
    QVBoxLayout *verticalLayout_3;
    QPushButton *transformButton;
    QGroupBox *files;
    QGridLayout *gridLayout_4;
    QPushButton *backgroundButton;
    QLabel *backgroundLabel;
    QComboBox *textureComboBox;
    QLabel *imageLabel;
    QPushButton *imageButton;
    QLabel *textureLabel;
    QComboBox *backgroundComboBox;
    QPushButton *textureButton;
    QComboBox *imageComboBox;
    QGroupBox *transformationType;
    QVBoxLayout *verticalLayout;
    QRadioButton *transformationTypeBRDF;
    QRadioButton *transformationTypeRetexture;
    QRadioButton *transformationTypeGlass;
    QRadioButton *transformationTypeCaustic;
    QRadioButton *transformationTypeGlossy;
    QRadioButton *transformationTypeExtra;
    QGroupBox *diffuseColor;
    QGridLayout *gridLayout_3;
    QLabel *diffuseColorLabelGreen;
    QLabel *diffuseColorLabelRed;
    QLineEdit *diffuseColorTextboxGreen;
    QLineEdit *diffuseColorTextboxBlue;
    QLabel *diffuseColorLabelBlue;
    QSlider *diffuseColorSliderGreen;
    QSlider *diffuseColorSliderRed;
    QSlider *diffuseColorSliderBlue;
    QLineEdit *diffuseColorTextboxRed;
    QGroupBox *specularColor;
    QGridLayout *gridLayout_19;
    QLabel *specularColorLabelBlue;
    QSlider *specularColorSliderBlue;
    QLabel *specularColorLabelGreen;
    QSlider *specularColorSliderGreen;
    QLabel *specularColorLabelRed;
    QLineEdit *specularColorTextboxRed;
    QSlider *specularColorSliderRed;
    QLineEdit *specularColorTextboxBlue;
    QLineEdit *specularColorTextboxGreen;
    QGroupBox *shapeEstimation;
    QGridLayout *gridLayout_20;
    QSlider *curvatureSlider;
    QSlider *smoothingSlider;
    QLineEdit *curvatureTextbox;
    QLabel *smoothingLabel;
    QLineEdit *smoothingTextbox;
    QLabel *curvatureLabel;
    QGroupBox *glass;
    QGridLayout *gridLayout_21;
    QLineEdit *frostyTextbox;
    QLineEdit *darkenssTextbox;
    QLabel *darknessLabel;
    QLabel *htLabel;
    QLineEdit *htTextbox;
    QSlider *darknessSlider;
    QLabel *sLabel;
    QSlider *htSlider;
    QLineEdit *sTextbox;
    QSlider *frostySlider;
    QLabel *frostyLabel;
    QSlider *sSlider;
    QWidget *emptySpaceBrush;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuToolbars;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(600, 3112);
        MainWindow->setDockNestingEnabled(true);
        MainWindow->setDockOptions(QMainWindow::AllowNestedDocks|QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks|QMainWindow::ForceTabbedDocks|QMainWindow::VerticalTabs);
        actionNew = new QAction(MainWindow);
        actionNew->setObjectName(QStringLiteral("actionNew"));
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName(QStringLiteral("actionSave"));
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName(QStringLiteral("actionQuit"));
        actionRevert = new QAction(MainWindow);
        actionRevert->setObjectName(QStringLiteral("actionRevert"));
        actionRevert->setEnabled(true);
        actionCopy3Dto2D = new QAction(MainWindow);
        actionCopy3Dto2D->setObjectName(QStringLiteral("actionCopy3Dto2D"));
        actionClear = new QAction(MainWindow);
        actionClear->setObjectName(QStringLiteral("actionClear"));
        actionUseOrbitingCamera = new QAction(MainWindow);
        actionUseOrbitingCamera->setObjectName(QStringLiteral("actionUseOrbitingCamera"));
        actionUseOrbitingCamera->setCheckable(true);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout_10 = new QGridLayout(centralWidget);
        gridLayout_10->setSpacing(6);
        gridLayout_10->setContentsMargins(11, 11, 11, 11);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        gridLayout_10->setContentsMargins(0, 0, 0, 0);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setTabPosition(QTabWidget::North);
        tab2D = new QWidget();
        tab2D->setObjectName(QStringLiteral("tab2D"));
        gridLayout_11 = new QGridLayout(tab2D);
        gridLayout_11->setSpacing(6);
        gridLayout_11->setContentsMargins(11, 11, 11, 11);
        gridLayout_11->setObjectName(QStringLiteral("gridLayout_11"));
        gridLayout_11->setContentsMargins(0, 0, 0, 0);
        scrollArea = new QScrollArea(tab2D);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setFrameShadow(QFrame::Plain);
        scrollArea->setLineWidth(0);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setWidgetResizable(true);
        canvas2D = new Canvas2D();
        canvas2D->setObjectName(QStringLiteral("canvas2D"));
        canvas2D->setGeometry(QRect(0, 0, 239, 3061));
        scrollArea->setWidget(canvas2D);

        gridLayout_11->addWidget(scrollArea, 0, 0, 1, 1);

        tabWidget->addTab(tab2D, QString());
        tab3D = new QWidget();
        tab3D->setObjectName(QStringLiteral("tab3D"));
        gridLayout_12 = new QGridLayout(tab3D);
        gridLayout_12->setSpacing(6);
        gridLayout_12->setContentsMargins(11, 11, 11, 11);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        gridLayout_12->setContentsMargins(0, 0, 0, 0);
        canvas3D = new QWidget(tab3D);
        canvas3D->setObjectName(QStringLiteral("canvas3D"));

        gridLayout_12->addWidget(canvas3D, 0, 0, 1, 1);

        tabWidget->addTab(tab3D, QString());

        gridLayout_10->addWidget(tabWidget, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        transformationDock = new QDockWidget(MainWindow);
        transformationDock->setObjectName(QStringLiteral("transformationDock"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(transformationDock->sizePolicy().hasHeightForWidth());
        transformationDock->setSizePolicy(sizePolicy);
        transformationDock->setFloating(false);
        transformationDock->setAllowedAreas(Qt::AllDockWidgetAreas);
        transformationDockContents = new QWidget();
        transformationDockContents->setObjectName(QStringLiteral("transformationDockContents"));
        sizePolicy.setHeightForWidth(transformationDockContents->sizePolicy().hasHeightForWidth());
        transformationDockContents->setSizePolicy(sizePolicy);
        verticalLayout_3 = new QVBoxLayout(transformationDockContents);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        transformButton = new QPushButton(transformationDockContents);
        transformButton->setObjectName(QStringLiteral("transformButton"));

        verticalLayout_3->addWidget(transformButton);

        files = new QGroupBox(transformationDockContents);
        files->setObjectName(QStringLiteral("files"));
        gridLayout_4 = new QGridLayout(files);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setVerticalSpacing(5);
        gridLayout_4->setContentsMargins(-1, 5, -1, 5);
        backgroundButton = new QPushButton(files);
        backgroundButton->setObjectName(QStringLiteral("backgroundButton"));

        gridLayout_4->addWidget(backgroundButton, 1, 2, 1, 1);

        backgroundLabel = new QLabel(files);
        backgroundLabel->setObjectName(QStringLiteral("backgroundLabel"));

        gridLayout_4->addWidget(backgroundLabel, 1, 0, 1, 1);

        textureComboBox = new QComboBox(files);
        textureComboBox->setObjectName(QStringLiteral("textureComboBox"));

        gridLayout_4->addWidget(textureComboBox, 3, 1, 1, 1);

        imageLabel = new QLabel(files);
        imageLabel->setObjectName(QStringLiteral("imageLabel"));

        gridLayout_4->addWidget(imageLabel, 0, 0, 1, 1);

        imageButton = new QPushButton(files);
        imageButton->setObjectName(QStringLiteral("imageButton"));

        gridLayout_4->addWidget(imageButton, 0, 2, 1, 1);

        textureLabel = new QLabel(files);
        textureLabel->setObjectName(QStringLiteral("textureLabel"));

        gridLayout_4->addWidget(textureLabel, 3, 0, 1, 1);

        backgroundComboBox = new QComboBox(files);
        backgroundComboBox->setObjectName(QStringLiteral("backgroundComboBox"));

        gridLayout_4->addWidget(backgroundComboBox, 1, 1, 1, 1);

        textureButton = new QPushButton(files);
        textureButton->setObjectName(QStringLiteral("textureButton"));

        gridLayout_4->addWidget(textureButton, 3, 2, 1, 1);

        imageComboBox = new QComboBox(files);
        imageComboBox->setObjectName(QStringLiteral("imageComboBox"));

        gridLayout_4->addWidget(imageComboBox, 0, 1, 1, 1);


        verticalLayout_3->addWidget(files);

        transformationType = new QGroupBox(transformationDockContents);
        transformationType->setObjectName(QStringLiteral("transformationType"));
        verticalLayout = new QVBoxLayout(transformationType);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(-1, 5, -1, 5);
        transformationTypeBRDF = new QRadioButton(transformationType);
        transformationTypeBRDF->setObjectName(QStringLiteral("transformationTypeBRDF"));
        transformationTypeBRDF->setChecked(true);

        verticalLayout->addWidget(transformationTypeBRDF);

        transformationTypeRetexture = new QRadioButton(transformationType);
        transformationTypeRetexture->setObjectName(QStringLiteral("transformationTypeRetexture"));

        verticalLayout->addWidget(transformationTypeRetexture);

        transformationTypeGlass = new QRadioButton(transformationType);
        transformationTypeGlass->setObjectName(QStringLiteral("transformationTypeGlass"));

        verticalLayout->addWidget(transformationTypeGlass);

        transformationTypeCaustic = new QRadioButton(transformationType);
        transformationTypeCaustic->setObjectName(QStringLiteral("transformationTypeCaustic"));

        verticalLayout->addWidget(transformationTypeCaustic);

        transformationTypeGlossy = new QRadioButton(transformationType);
        transformationTypeGlossy->setObjectName(QStringLiteral("transformationTypeGlossy"));

        verticalLayout->addWidget(transformationTypeGlossy);

        transformationTypeExtra = new QRadioButton(transformationType);
        transformationTypeExtra->setObjectName(QStringLiteral("transformationTypeExtra"));

        verticalLayout->addWidget(transformationTypeExtra);


        verticalLayout_3->addWidget(transformationType);

        diffuseColor = new QGroupBox(transformationDockContents);
        diffuseColor->setObjectName(QStringLiteral("diffuseColor"));
        gridLayout_3 = new QGridLayout(diffuseColor);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setVerticalSpacing(5);
        gridLayout_3->setContentsMargins(-1, 5, -1, 5);
        diffuseColorLabelGreen = new QLabel(diffuseColor);
        diffuseColorLabelGreen->setObjectName(QStringLiteral("diffuseColorLabelGreen"));

        gridLayout_3->addWidget(diffuseColorLabelGreen, 1, 0, 1, 1);

        diffuseColorLabelRed = new QLabel(diffuseColor);
        diffuseColorLabelRed->setObjectName(QStringLiteral("diffuseColorLabelRed"));

        gridLayout_3->addWidget(diffuseColorLabelRed, 0, 0, 1, 1);

        diffuseColorTextboxGreen = new QLineEdit(diffuseColor);
        diffuseColorTextboxGreen->setObjectName(QStringLiteral("diffuseColorTextboxGreen"));
        diffuseColorTextboxGreen->setMinimumSize(QSize(50, 0));
        diffuseColorTextboxGreen->setMaximumSize(QSize(50, 16777215));

        gridLayout_3->addWidget(diffuseColorTextboxGreen, 1, 3, 1, 1);

        diffuseColorTextboxBlue = new QLineEdit(diffuseColor);
        diffuseColorTextboxBlue->setObjectName(QStringLiteral("diffuseColorTextboxBlue"));
        diffuseColorTextboxBlue->setMinimumSize(QSize(50, 0));
        diffuseColorTextboxBlue->setMaximumSize(QSize(50, 16777215));

        gridLayout_3->addWidget(diffuseColorTextboxBlue, 2, 3, 1, 1);

        diffuseColorLabelBlue = new QLabel(diffuseColor);
        diffuseColorLabelBlue->setObjectName(QStringLiteral("diffuseColorLabelBlue"));

        gridLayout_3->addWidget(diffuseColorLabelBlue, 2, 0, 1, 1);

        diffuseColorSliderGreen = new QSlider(diffuseColor);
        diffuseColorSliderGreen->setObjectName(QStringLiteral("diffuseColorSliderGreen"));
        diffuseColorSliderGreen->setMinimumSize(QSize(100, 0));
        diffuseColorSliderGreen->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(diffuseColorSliderGreen, 1, 2, 1, 1);

        diffuseColorSliderRed = new QSlider(diffuseColor);
        diffuseColorSliderRed->setObjectName(QStringLiteral("diffuseColorSliderRed"));
        diffuseColorSliderRed->setMinimumSize(QSize(100, 0));
        diffuseColorSliderRed->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(diffuseColorSliderRed, 0, 2, 1, 1);

        diffuseColorSliderBlue = new QSlider(diffuseColor);
        diffuseColorSliderBlue->setObjectName(QStringLiteral("diffuseColorSliderBlue"));
        diffuseColorSliderBlue->setMinimumSize(QSize(100, 0));
        diffuseColorSliderBlue->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(diffuseColorSliderBlue, 2, 2, 1, 1);

        diffuseColorTextboxRed = new QLineEdit(diffuseColor);
        diffuseColorTextboxRed->setObjectName(QStringLiteral("diffuseColorTextboxRed"));
        diffuseColorTextboxRed->setMinimumSize(QSize(50, 0));
        diffuseColorTextboxRed->setMaximumSize(QSize(50, 16777215));

        gridLayout_3->addWidget(diffuseColorTextboxRed, 0, 3, 1, 1);


        verticalLayout_3->addWidget(diffuseColor);

        specularColor = new QGroupBox(transformationDockContents);
        specularColor->setObjectName(QStringLiteral("specularColor"));
        gridLayout_19 = new QGridLayout(specularColor);
        gridLayout_19->setSpacing(6);
        gridLayout_19->setContentsMargins(11, 11, 11, 11);
        gridLayout_19->setObjectName(QStringLiteral("gridLayout_19"));
        gridLayout_19->setVerticalSpacing(5);
        gridLayout_19->setContentsMargins(-1, 5, -1, 5);
        specularColorLabelBlue = new QLabel(specularColor);
        specularColorLabelBlue->setObjectName(QStringLiteral("specularColorLabelBlue"));

        gridLayout_19->addWidget(specularColorLabelBlue, 2, 0, 1, 1);

        specularColorSliderBlue = new QSlider(specularColor);
        specularColorSliderBlue->setObjectName(QStringLiteral("specularColorSliderBlue"));
        specularColorSliderBlue->setMinimumSize(QSize(100, 0));
        specularColorSliderBlue->setOrientation(Qt::Horizontal);

        gridLayout_19->addWidget(specularColorSliderBlue, 2, 2, 1, 1);

        specularColorLabelGreen = new QLabel(specularColor);
        specularColorLabelGreen->setObjectName(QStringLiteral("specularColorLabelGreen"));

        gridLayout_19->addWidget(specularColorLabelGreen, 1, 0, 1, 1);

        specularColorSliderGreen = new QSlider(specularColor);
        specularColorSliderGreen->setObjectName(QStringLiteral("specularColorSliderGreen"));
        specularColorSliderGreen->setMinimumSize(QSize(100, 0));
        specularColorSliderGreen->setOrientation(Qt::Horizontal);

        gridLayout_19->addWidget(specularColorSliderGreen, 1, 2, 1, 1);

        specularColorLabelRed = new QLabel(specularColor);
        specularColorLabelRed->setObjectName(QStringLiteral("specularColorLabelRed"));

        gridLayout_19->addWidget(specularColorLabelRed, 0, 0, 1, 1);

        specularColorTextboxRed = new QLineEdit(specularColor);
        specularColorTextboxRed->setObjectName(QStringLiteral("specularColorTextboxRed"));
        specularColorTextboxRed->setMinimumSize(QSize(50, 0));
        specularColorTextboxRed->setMaximumSize(QSize(50, 16777215));

        gridLayout_19->addWidget(specularColorTextboxRed, 0, 3, 1, 1);

        specularColorSliderRed = new QSlider(specularColor);
        specularColorSliderRed->setObjectName(QStringLiteral("specularColorSliderRed"));
        specularColorSliderRed->setMinimumSize(QSize(100, 0));
        specularColorSliderRed->setOrientation(Qt::Horizontal);

        gridLayout_19->addWidget(specularColorSliderRed, 0, 2, 1, 1);

        specularColorTextboxBlue = new QLineEdit(specularColor);
        specularColorTextboxBlue->setObjectName(QStringLiteral("specularColorTextboxBlue"));
        specularColorTextboxBlue->setMinimumSize(QSize(50, 0));
        specularColorTextboxBlue->setMaximumSize(QSize(50, 16777215));

        gridLayout_19->addWidget(specularColorTextboxBlue, 2, 3, 1, 1);

        specularColorTextboxGreen = new QLineEdit(specularColor);
        specularColorTextboxGreen->setObjectName(QStringLiteral("specularColorTextboxGreen"));
        specularColorTextboxGreen->setMinimumSize(QSize(50, 0));
        specularColorTextboxGreen->setMaximumSize(QSize(50, 16777215));

        gridLayout_19->addWidget(specularColorTextboxGreen, 1, 3, 1, 1);


        verticalLayout_3->addWidget(specularColor);

        shapeEstimation = new QGroupBox(transformationDockContents);
        shapeEstimation->setObjectName(QStringLiteral("shapeEstimation"));
        gridLayout_20 = new QGridLayout(shapeEstimation);
        gridLayout_20->setSpacing(6);
        gridLayout_20->setContentsMargins(11, 11, 11, 11);
        gridLayout_20->setObjectName(QStringLiteral("gridLayout_20"));
        gridLayout_20->setVerticalSpacing(5);
        gridLayout_20->setContentsMargins(-1, 5, -1, 5);
        curvatureSlider = new QSlider(shapeEstimation);
        curvatureSlider->setObjectName(QStringLiteral("curvatureSlider"));
        curvatureSlider->setMinimumSize(QSize(100, 0));
        curvatureSlider->setOrientation(Qt::Horizontal);

        gridLayout_20->addWidget(curvatureSlider, 0, 2, 1, 1);

        smoothingSlider = new QSlider(shapeEstimation);
        smoothingSlider->setObjectName(QStringLiteral("smoothingSlider"));
        smoothingSlider->setMinimumSize(QSize(100, 0));
        smoothingSlider->setOrientation(Qt::Horizontal);

        gridLayout_20->addWidget(smoothingSlider, 1, 2, 1, 1);

        curvatureTextbox = new QLineEdit(shapeEstimation);
        curvatureTextbox->setObjectName(QStringLiteral("curvatureTextbox"));
        curvatureTextbox->setMinimumSize(QSize(50, 0));
        curvatureTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_20->addWidget(curvatureTextbox, 0, 3, 1, 1);

        smoothingLabel = new QLabel(shapeEstimation);
        smoothingLabel->setObjectName(QStringLiteral("smoothingLabel"));

        gridLayout_20->addWidget(smoothingLabel, 1, 0, 1, 1);

        smoothingTextbox = new QLineEdit(shapeEstimation);
        smoothingTextbox->setObjectName(QStringLiteral("smoothingTextbox"));
        smoothingTextbox->setMinimumSize(QSize(50, 0));
        smoothingTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_20->addWidget(smoothingTextbox, 1, 3, 1, 1);

        curvatureLabel = new QLabel(shapeEstimation);
        curvatureLabel->setObjectName(QStringLiteral("curvatureLabel"));

        gridLayout_20->addWidget(curvatureLabel, 0, 0, 1, 1);


        verticalLayout_3->addWidget(shapeEstimation);

        glass = new QGroupBox(transformationDockContents);
        glass->setObjectName(QStringLiteral("glass"));
        gridLayout_21 = new QGridLayout(glass);
        gridLayout_21->setSpacing(6);
        gridLayout_21->setContentsMargins(11, 11, 11, 11);
        gridLayout_21->setObjectName(QStringLiteral("gridLayout_21"));
        gridLayout_21->setVerticalSpacing(5);
        gridLayout_21->setContentsMargins(-1, 5, -1, 5);
        frostyTextbox = new QLineEdit(glass);
        frostyTextbox->setObjectName(QStringLiteral("frostyTextbox"));
        frostyTextbox->setMinimumSize(QSize(50, 0));
        frostyTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_21->addWidget(frostyTextbox, 1, 3, 1, 1);

        darkenssTextbox = new QLineEdit(glass);
        darkenssTextbox->setObjectName(QStringLiteral("darkenssTextbox"));
        darkenssTextbox->setMinimumSize(QSize(50, 0));
        darkenssTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_21->addWidget(darkenssTextbox, 2, 3, 1, 1);

        darknessLabel = new QLabel(glass);
        darknessLabel->setObjectName(QStringLiteral("darknessLabel"));

        gridLayout_21->addWidget(darknessLabel, 2, 0, 1, 1);

        htLabel = new QLabel(glass);
        htLabel->setObjectName(QStringLiteral("htLabel"));

        gridLayout_21->addWidget(htLabel, 3, 0, 1, 1);

        htTextbox = new QLineEdit(glass);
        htTextbox->setObjectName(QStringLiteral("htTextbox"));
        htTextbox->setMinimumSize(QSize(50, 0));
        htTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_21->addWidget(htTextbox, 3, 3, 1, 1);

        darknessSlider = new QSlider(glass);
        darknessSlider->setObjectName(QStringLiteral("darknessSlider"));
        darknessSlider->setMinimumSize(QSize(100, 0));
        darknessSlider->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(darknessSlider, 2, 2, 1, 1);

        sLabel = new QLabel(glass);
        sLabel->setObjectName(QStringLiteral("sLabel"));

        gridLayout_21->addWidget(sLabel, 0, 0, 1, 1);

        htSlider = new QSlider(glass);
        htSlider->setObjectName(QStringLiteral("htSlider"));
        htSlider->setMinimumSize(QSize(100, 0));
        htSlider->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(htSlider, 3, 2, 1, 1);

        sTextbox = new QLineEdit(glass);
        sTextbox->setObjectName(QStringLiteral("sTextbox"));
        sTextbox->setMinimumSize(QSize(50, 0));
        sTextbox->setMaximumSize(QSize(50, 16777215));

        gridLayout_21->addWidget(sTextbox, 0, 3, 1, 1);

        frostySlider = new QSlider(glass);
        frostySlider->setObjectName(QStringLiteral("frostySlider"));
        frostySlider->setMinimumSize(QSize(100, 0));
        frostySlider->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(frostySlider, 1, 2, 1, 1);

        frostyLabel = new QLabel(glass);
        frostyLabel->setObjectName(QStringLiteral("frostyLabel"));

        gridLayout_21->addWidget(frostyLabel, 1, 0, 1, 1);

        sSlider = new QSlider(glass);
        sSlider->setObjectName(QStringLiteral("sSlider"));
        sSlider->setMinimumSize(QSize(100, 0));
        sSlider->setOrientation(Qt::Horizontal);

        gridLayout_21->addWidget(sSlider, 0, 2, 1, 1);


        verticalLayout_3->addWidget(glass);

        emptySpaceBrush = new QWidget(transformationDockContents);
        emptySpaceBrush->setObjectName(QStringLiteral("emptySpaceBrush"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(emptySpaceBrush->sizePolicy().hasHeightForWidth());
        emptySpaceBrush->setSizePolicy(sizePolicy1);

        verticalLayout_3->addWidget(emptySpaceBrush);

        transformationDock->setWidget(transformationDockContents);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(1), transformationDock);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 22));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuToolbars = new QMenu(menuBar);
        menuToolbars->setObjectName(QStringLiteral("menuToolbars"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuToolbars->menuAction());
        menuFile->addAction(actionNew);
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionRevert);
        menuFile->addAction(actionClear);
        menuFile->addAction(actionCopy3Dto2D);
        menuFile->addAction(actionUseOrbitingCamera);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);

        retranslateUi(MainWindow);
        QObject::connect(actionQuit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(actionSave, SIGNAL(triggered()), MainWindow, SLOT(fileSave()));
        QObject::connect(actionOpen, SIGNAL(triggered()), MainWindow, SLOT(fileOpen()));
        QObject::connect(actionNew, SIGNAL(triggered()), MainWindow, SLOT(fileNew()));
        QObject::connect(actionCopy3Dto2D, SIGNAL(triggered()), MainWindow, SLOT(fileCopy3Dto2D()));
        QObject::connect(actionClear, SIGNAL(triggered()), MainWindow, SLOT(clearImage()));
        QObject::connect(actionRevert, SIGNAL(triggered()), MainWindow, SLOT(revertImage()));

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "CS 123: Introduction to Computer Graphics", Q_NULLPTR));
        actionNew->setText(QApplication::translate("MainWindow", "&New", Q_NULLPTR));
        actionNew->setShortcut(QApplication::translate("MainWindow", "Ctrl+N", Q_NULLPTR));
        actionOpen->setText(QApplication::translate("MainWindow", "&Open...", Q_NULLPTR));
        actionOpen->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", Q_NULLPTR));
        actionSave->setText(QApplication::translate("MainWindow", "&Save...", Q_NULLPTR));
        actionSave->setShortcut(QApplication::translate("MainWindow", "Ctrl+S", Q_NULLPTR));
        actionQuit->setText(QApplication::translate("MainWindow", "&Quit", Q_NULLPTR));
        actionQuit->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", Q_NULLPTR));
        actionRevert->setText(QApplication::translate("MainWindow", "&Revert Image", Q_NULLPTR));
        actionRevert->setShortcut(QApplication::translate("MainWindow", "Ctrl+R", Q_NULLPTR));
        actionCopy3Dto2D->setText(QApplication::translate("MainWindow", "Copy &3D tab to 2D tab", Q_NULLPTR));
        actionClear->setText(QApplication::translate("MainWindow", "&Clear Image", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionClear->setToolTip(QApplication::translate("MainWindow", "Clear Image", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionClear->setShortcut(QApplication::translate("MainWindow", "Ctrl+C", Q_NULLPTR));
        actionUseOrbitingCamera->setText(QApplication::translate("MainWindow", "&Use Orbiting Camera", Q_NULLPTR));
        actionUseOrbitingCamera->setShortcut(QApplication::translate("MainWindow", "Ctrl+B", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab2D), QApplication::translate("MainWindow", "2D", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tab3D), QApplication::translate("MainWindow", "3D", Q_NULLPTR));
        transformationDock->setWindowTitle(QApplication::translate("MainWindow", "Br&ush", Q_NULLPTR));
        transformButton->setText(QApplication::translate("MainWindow", "Transform", Q_NULLPTR));
        files->setTitle(QApplication::translate("MainWindow", "Files", Q_NULLPTR));
        backgroundButton->setText(QApplication::translate("MainWindow", "Find", Q_NULLPTR));
        backgroundLabel->setText(QApplication::translate("MainWindow", "Browse Background", Q_NULLPTR));
        imageLabel->setText(QApplication::translate("MainWindow", "Browse Image", Q_NULLPTR));
        imageButton->setText(QApplication::translate("MainWindow", "Find", Q_NULLPTR));
        textureLabel->setText(QApplication::translate("MainWindow", "Browse Texture", Q_NULLPTR));
        textureButton->setText(QApplication::translate("MainWindow", "Find", Q_NULLPTR));
        transformationType->setTitle(QApplication::translate("MainWindow", "Transformation Type", Q_NULLPTR));
        transformationTypeBRDF->setText(QApplication::translate("MainWindow", "BRDF", Q_NULLPTR));
        transformationTypeRetexture->setText(QApplication::translate("MainWindow", "Retexture", Q_NULLPTR));
        transformationTypeGlass->setText(QApplication::translate("MainWindow", "Glass", Q_NULLPTR));
        transformationTypeCaustic->setText(QApplication::translate("MainWindow", "Caustic", Q_NULLPTR));
        transformationTypeGlossy->setText(QApplication::translate("MainWindow", "Glossy", Q_NULLPTR));
        transformationTypeExtra->setText(QApplication::translate("MainWindow", "Extra", Q_NULLPTR));
        diffuseColor->setTitle(QApplication::translate("MainWindow", "Diffuse", Q_NULLPTR));
        diffuseColorLabelGreen->setText(QApplication::translate("MainWindow", "G", Q_NULLPTR));
        diffuseColorLabelRed->setText(QApplication::translate("MainWindow", "R", Q_NULLPTR));
        diffuseColorLabelBlue->setText(QApplication::translate("MainWindow", "B", Q_NULLPTR));
        specularColor->setTitle(QApplication::translate("MainWindow", "Specular", Q_NULLPTR));
        specularColorLabelBlue->setText(QApplication::translate("MainWindow", "B", Q_NULLPTR));
        specularColorLabelGreen->setText(QApplication::translate("MainWindow", "G", Q_NULLPTR));
        specularColorLabelRed->setText(QApplication::translate("MainWindow", "R", Q_NULLPTR));
        shapeEstimation->setTitle(QApplication::translate("MainWindow", "Shape Estimation", Q_NULLPTR));
        smoothingLabel->setText(QApplication::translate("MainWindow", "Smoothing", Q_NULLPTR));
        curvatureLabel->setText(QApplication::translate("MainWindow", "Curvature", Q_NULLPTR));
        glass->setTitle(QApplication::translate("MainWindow", "Glass", Q_NULLPTR));
        darknessLabel->setText(QApplication::translate("MainWindow", "Darkness", Q_NULLPTR));
        htLabel->setText(QApplication::translate("MainWindow", "HT", Q_NULLPTR));
        sLabel->setText(QApplication::translate("MainWindow", "S", Q_NULLPTR));
        frostyLabel->setText(QApplication::translate("MainWindow", "Frosty", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "&File", Q_NULLPTR));
        menuToolbars->setTitle(QApplication::translate("MainWindow", "&Toolbars", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
