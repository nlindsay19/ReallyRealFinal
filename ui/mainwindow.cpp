#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Databinding.h"
#include "CS123XmlSceneParser.h"
#include "scenegraph/RayScene.h"
#include "CS123XmlSceneParser.h"
#include <math.h>
#include <QFileDialog>
#include <QMessageBox>
#include <iostream>
#include <Settings.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // Make sure the settings are loaded before the UI
    settings.loadSettingsOrDefaults();

////    QGLFormat qglFormat;
//    qglFormat.setVersion(4, 3);
//    qglFormat.setProfile(QGLFormat::CoreProfile);
//    qglFormat.setSampleBuffers(true);
    ui->setupUi(this);
//    QGridLayout *gridLayout = new QGridLayout(ui->canvas3D);
//    m_canvas3D = new SupportCanvas3D(qglFormat, this);
//    gridLayout->addWidget(m_canvas3D, 0, 1);
    ui->tabWidget->setCurrentWidget(ui->tab2D);

    // Restore the UI settings
    QSettings qtSettings("CS123", "CS123");
    restoreGeometry(qtSettings.value("geometry").toByteArray());
    restoreState(qtSettings.value("windowState").toByteArray());

    // Resize the window because the window is huge since all docks were visible.
    resize(1000, 600);

    // Bind the UI elements to their properties in the global Settings object, using binding
    // objects to simplify the code.  Each binding object connects to its UI elements and keeps
    // the UI and its setting in sync.

    QList<QAction*> actions;

#define SETUP_ACTION(dock, key) \
    actions.push_back(ui->dock->toggleViewAction()); \
    actions.back()->setShortcut(QKeySequence(key));

    SETUP_ACTION(transformationDock,     "CTRL+1");

    ui->menuToolbars->addActions(actions);
#undef SETUP_ACTION

    ui->transformationDock->raise();

    dataBind();



    // Reset the contents of both canvas widgets (make a new 500x500 image for the 2D one)
    fileNew();

    // Make certain radio buttons switch to the 2D canvas when clicked.
    QList<QRadioButton*> a;
    a += ui->transformationTypeRetexture;
    a += ui->transformationTypeGlass;
    a += ui->transformationTypeCaustic;
    a += ui->transformationTypeBRDF;
    a += ui->transformationTypeGlossy;
    a += ui->transformationTypeExtra;
    foreach (QRadioButton *rb, a)
        connect(rb, SIGNAL(clicked()), this, SLOT(activateCanvas2D()));

    a.clear();
//    foreach (QRadioButton *rb, a)
//        connect(rb, SIGNAL(clicked()), this, SLOT(activateCanvas3D()));

//    QWidget *widget = ui->tabWidget->currentWidget();
//    ui->tabWidget->setCurrentWidget(ui->tab2D);
//    show();
//    ui->tabWidget->setCurrentWidget(widget);
//    show();

}

MainWindow::~MainWindow()
{
    foreach (DataBinding *b, m_bindings)
        delete b;
    foreach (QButtonGroup *bg, m_buttonGroups)
        delete bg;
    delete ui;
}

void MainWindow::dataBind() {
    // Brush dock
#define BIND(b) { \
    DataBinding *_b = (b); \
    m_bindings.push_back(_b); \
    assert(connect(_b, SIGNAL(dataChanged()), this, SLOT(settingsChanged()))); \
}
    connect(ui->transformButton, SIGNAL (released()),this, SLOT (transformPressed()));
    QButtonGroup *transformationButtonGroup = new QButtonGroup;
    m_buttonGroups.push_back(transformationButtonGroup);

    BIND(ChoiceBinding::bindRadioButtons(
            transformationButtonGroup,
            NUM_BRUSH_TYPES,
            settings.transformationType,
            ui->transformationTypeBRDF,
            ui->transformationTypeRetexture,
            ui->transformationTypeGlass,
            ui->transformationTypeCaustic,
            ui->transformationTypeGlossy,
            ui->transformationTypeExtra))

    // TODO: Bind push button?
    // BIND(ChoiceBinding::bind)

    // Diffuse Sliders
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->diffuseColorSliderRed, ui->diffuseColorTextboxRed, settings.diffuseColor.r, 0, 255))
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->diffuseColorSliderGreen, ui->diffuseColorTextboxGreen, settings.diffuseColor.g, 0, 255))
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->diffuseColorSliderBlue, ui->diffuseColorTextboxBlue, settings.diffuseColor.b, 0, 255))
    // Diffuse Sliders
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->specularColorSliderRed, ui->specularColorTextboxRed, settings.specularColor.r, 0, 255))
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->specularColorSliderGreen, ui->specularColorTextboxGreen, settings.specularColor.g, 0, 255))
    BIND(UCharBinding::bindSliderAndTextbox(
        ui->specularColorSliderBlue, ui->specularColorTextboxBlue, settings.specularColor.b, 0, 255))
    // Shape Estimation
    BIND(FloatBinding::bindSliderAndTextbox(
        ui->smoothingSlider, ui->smoothingTextbox, settings.smoothing, 0, 1))
    BIND(IntBinding::bindSliderAndTextbox(
        ui->curvatureSlider, ui->curvatureTextbox, settings.curvature, 0, 10))
    // Glass
    BIND(FloatBinding::bindSliderAndTextbox(
        ui->sSlider, ui->sTextbox, settings.sValue, 0, 1))
    BIND(IntBinding::bindSliderAndTextbox(
        ui->frostySlider, ui->frostyTextbox, settings.frosty, 0, 50))
    BIND(IntBinding::bindSliderAndTextbox(
        ui->darknessSlider, ui->darkenssTextbox, settings.darkness, 1, 5))
    BIND(FloatBinding::bindSliderAndTextbox(
        ui->htSlider, ui->htTextbox, settings.ht, 0, 1))

//    BIND(BoolBinding::bindCheckbox(ui->brushAlphaBlendingCheckbox, settings.fixAlphaBlending))

#undef BIND

    // make sure the aspect ratio updates when m_canvas3D changes size
//    connect(m_canvas3D, SIGNAL(aspectRatioChanged()), this, SLOT(updateAspectRatio()));
}

void MainWindow::changeEvent(QEvent *e) {
    QMainWindow::changeEvent(e); // allow the superclass to handle this for the most part...

    switch (e->type()) {
        case QEvent::LanguageChange:
            ui->retranslateUi(this);
            break;
        default:
            break;
    }
}

void MainWindow::closeEvent(QCloseEvent *event) {
    // Save the settings before we quit
    settings.saveSettings();
    QSettings qtSettings("CS123", "CS123");
    qtSettings.setValue("geometry", saveGeometry());
    qtSettings.setValue("windowState", saveState());

    // Stop any raytracing, otherwise the thread will hang around until done
    ui->canvas2D->cancelRender();

    QMainWindow::closeEvent(event);
}

void MainWindow::updateAspectRatio() {
    // Update the aspect ratio text so the aspect ratio will be correct even if the
    // 3D canvas isn't visible (the 3D canvas isn't resized when it isn't visible)
    QSize activeTabSize = ui->tabWidget->currentWidget()->size();
    float aspectRatio = static_cast<float>(activeTabSize.width()) / static_cast<float>(activeTabSize.height());
}


void MainWindow::settingsChanged() {
    ui->canvas2D->settingsChanged();
    std::cout << "settings changed" << std::endl;
}

void MainWindow::transformPressed() {
    std::cout << "autobots roll out" << std::endl;
}


void MainWindow::fileCopy3Dto2D() {
//    // Make sure OpenGL gets a chance to redraw
//    ui->tabWidget->setCurrentIndex(TAB_3D);
//    m_canvas3D->update();
//    QApplication::processEvents();

//    // Resize the 2D canvas to the size of the 3D canvas and copy the pixels over.
//    float ratio = static_cast<QGuiApplication *>(QCoreApplication::instance())->devicePixelRatio();
//    ui->canvas2D->resize(m_canvas3D->width() * ratio, m_canvas3D->height() * ratio);
//    m_canvas3D->copyPixels(ui->canvas2D->width(), ui->canvas2D->height(), ui->canvas2D->data());
//    ui->tabWidget->setCurrentIndex(TAB_2D);
}

void MainWindow::fileNew() {
    ui->canvas2D->newImage();
}

void MainWindow::fileOpen() {
    QString file = QFileDialog::getOpenFileName(this, QString(), "/course/cs123/data/");
    if (!file.isNull()) {
        if (!ui->canvas2D->loadImage(file)) {
            QMessageBox::critical(this, "Error", "Could not load image \"" + file + "\"");
        } else {
            activateCanvas2D();
        }
    }
}

void MainWindow::fileSave() {
    if (settings.currentTab == TAB_2D)
        ui->canvas2D->saveImage();
}

void MainWindow::filterImage() {
    // Disable the UI so the user can't interfere with the filtering
    setAllEnabled(false);

    // Actually do the filter.
    ui->canvas2D->filterImage();

    // Enable the UI again
    setAllEnabled(true);
}

void MainWindow::renderImage() {
//    // Make sure OpenGL gets a chance to update the OrbitCamera, which can only be done when
//    // that tab is active (because it needs the OpenGL context for its matrix transforms)
//    ui->tabWidget->setCurrentIndex(TAB_3D);
//    m_canvas3D->update();
//    QApplication::processEvents();

//    ui->tabWidget->setCurrentIndex(TAB_2D);

//    OpenGLScene *glScene = m_canvas3D->getScene();
//    if (glScene) {
//        // TODO: Set up RayScene from glScene and call ui->canvas2D->setScene()

//        // Disable the UI so the user can't interfere with the raytracing
//        setAllEnabled(false);

//        // Render the image
//        QSize activeTabSize = ui->tabWidget->currentWidget()->size();
//        ui->canvas2D->renderImage(m_canvas3D->getCamera(), activeTabSize.width(), activeTabSize.height());

//        // Enable the UI again
//        setAllEnabled(true);
//    }
}

void MainWindow::setAllEnabled(bool enabled) {
    QList<QWidget *> widgets;
    widgets += ui->transformationDock;

    QList<QAction *> actions;
    actions += ui->actionNew;
    actions += ui->actionOpen;
    actions += ui->actionSave;
    actions += ui->actionRevert;
//    actions += ui->actionCopy3Dto2D;
    actions += ui->actionClear;
    actions += ui->actionQuit;

    foreach (QWidget *widget, widgets)
        widget->setEnabled(enabled);
    foreach (QAction *action, actions)
        action->setEnabled(enabled);
}

void MainWindow::activateCanvas2D() {
    ui->tabWidget->setCurrentWidget(ui->tab2D);
}

void MainWindow::clearImage()
{
    ui->canvas2D->clearImage();
}

void MainWindow::revertImage()
{
    ui->canvas2D->revertImage();
}
