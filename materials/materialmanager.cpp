#include "materialmanager.h"
#include "shapeestimation.h"
#include "incidentlight.h"
#include "brdfreplacement.h"
#include "retexture.h"
#include "histogram.h"
#include "causticmaker.h"
#include <QProcess>
#include <QProcessEnvironment>
#include <iostream>
#include <fstream>
#include <cmath>

MaterialManager::MaterialManager()
{

}

bool MaterialManager::transformMaterial(){
    if(materialParams.makeMaterial == BRDF){
        return changeBrdf();
    }
    if(materialParams.makeMaterial == RETEXTURE){
        return retexture();
    }
    if(materialParams.makeMaterial == GLASS){
        makeGlass();

        std::cout << "make glass" << std::endl;
        return true;
    }
    if(materialParams.makeMaterial == CAUSTIC){
        return makeCaustic();
    }
    if(materialParams.makeMaterial == LIGHTING){
        return changeLighting();
    }
    if(materialParams.makeMaterial == GLOSSY) {
        return makeGlossy(1);
    }
}

bool MaterialManager::areBasicParamsValid(){
    if(materialParams.mainImageFile == ""){
        std::cout << "Please insert a main image file" << std::endl;
        return false;
    }
    if(materialParams.maskFile == ""){
        std::cout << "Please insert a mask image file" << std::endl;
        return false;
    }
    return true;
}

bool MaterialManager::areBrdfParamsValid(){
    if(materialParams.diffuse[0] < 0 || materialParams.diffuse[1] < 0 || materialParams.diffuse[2] < 0){
        std::cout << "Invalid diffuse values" << std::endl;
        return false;
    }
    if(materialParams.specular[0] < 0 || materialParams.specular[1] < 0 || materialParams.specular[2] < 0){
        std::cout << "Invalid specular values" << std::endl;
        return false;
    }
    return true;
}

bool MaterialManager::areShapeEstimationParamsValid(){
    if(materialParams.bilateralSmoothing < 0){
        std::cout << "invalid bilateral smoothing value" << std::endl;
        return false;
    }
    if(materialParams.curvature < 0){
        std::cout << "invalid curvature values" << std::endl;
        return false;
    }

    return true;
}

bool MaterialManager::areRetextureParamsValid(){
    if(materialParams.textureFile == ""){
        std::cout << "invalid texture file" << std::endl;
        return false;
    }
    if(materialParams.s < 0){
        std::cout << "roughness value invalid" << std::endl;
        return false;
    }
    return true;
}

bool MaterialManager::areGlassParametersValid(){
    if(materialParams.frosty < 0){
        std::cout << "frosted glass value is invalid" << std::endl;
        return false;
    }
    if(materialParams.darkness < 0){
        std::cout << " glass darkness value is invalid" << std::endl;
        return false;
    }
    if(materialParams.glassColor[0] < 0 || materialParams.glassColor[1] < 0 || materialParams.glassColor[2] < 0){
        std::cout << " glass color value is invalid" << std::endl;
        return false;
    }
    return true;
}

bool MaterialManager::changeBrdf(){
    if(!areBasicParamsValid()){
        return false;
    }
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    if(materialParams.backgroundFile == ""){
        // Use input image as background rather than replacing background.
        materialParams.backgroundFile = materialParams.mainImageFile;
    }
    ImageReader background(materialParams.backgroundFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;

    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);

    incidentlight incidentObj;
    std::vector<Vector3f> inpainting = incidentObj.inPaint(mask, background.toVector());
    vectorToFile(inpainting, "images/inpaint.png", rows, cols);

    BrdfReplacement br;

    if(!areBrdfParamsValid()){
        return false;
    }
    br.m_diffuse = materialParams.diffuse;
    br.m_specular = materialParams.specular;

    std::vector<Vector3f> replaced = br.replaceBrdf(inpainting, mask.toVector(), normals, rows, cols);
    vectorToFile(replaced, "images/output.png", rows, cols);
    return true;
}

bool MaterialManager::changeLighting(){
    if(!areBasicParamsValid()){
        return false;
    }
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    if(materialParams.backgroundFile == ""){
        // Use input image as background rather than replacing background.
        materialParams.backgroundFile = materialParams.mainImageFile;
    }
    ImageReader background(materialParams.backgroundFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;

    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);

    incidentlight incidentObj;
    std::vector<Vector3f> inpainting = incidentObj.inPaint(mask, background.toVector());
    vectorToFile(inpainting, "images/inpaint.png", rows, cols);

    BrdfReplacement br;

    if(!areBrdfParamsValid()){
        return false;
    }
    br.m_diffuse = Vector3f(0.0f,0.0f,0.0f);
    br.m_specular = Vector3f(1.0f,1.0f,1.0f);

    std::vector<Vector3f> replaced = br.paintEnvMap(inpainting, mask.toVector(), normals, rows, cols, materialParams.desiredColors, materialParams.highlight);
    materialResults.image = replaced;
    materialResults.mask = mask.toVector();
    materialResults.normals = normals;
    materialResults.specularDirs = br.specularDirs;
    materialResults.rows = rows;
    materialResults.cols = cols;
    br.addHighlightsToEnvmap(replaced, mask.toVector(), normals, rows, cols, materialParams.highlightColors, materialParams.highlight, materialParams.s * 2.0f);

    vectorToFile(replaced, "images/output.png", rows, cols);

    for(int i = 0; i < materialParams.desiredColors.size(); i++){
        materialParams.desiredColors[i] = Vector3f(0,0,0);
    }
    materialParams.highlight = Vector2f(-1.0f,-1.0f);
    return true;
}

void MaterialManager::vectorToFile(std::vector<Vector3f> data, QString filenameOut, int rows, int cols){
    QImage imageOut(cols, rows, QImage::Format_RGB32);
    QRgb *imageBits = reinterpret_cast<QRgb *>(imageOut.bits());
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            int index = i * cols + j;
            Vector3f color = data[index];
            float colorR = fmin(color(0),255);
            float colorG = fmin(color(1), 255);
            float colorB = fmin(color(2), 255);
            QColor colorOut = QColor(int(colorR), int(colorG), int(colorB));
            imageBits[index] = colorOut.rgb();
        }
    }
    imageOut.save(filenameOut);
}

bool MaterialManager::retexture(){
    if(!areBasicParamsValid()){
        return false;
    }
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    if(materialParams.backgroundFile == ""){
        // Use input image as background rather than replacing background.
        materialParams.backgroundFile = materialParams.mainImageFile;
    }
    ImageReader background(materialParams.backgroundFile);

    if(!areRetextureParamsValid()){
        return false;
    }
    ImageReader texture(materialParams.textureFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;

    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);

    incidentlight incidentObj;
    std::vector<Vector3f> inpainting = incidentObj.inPaint(mask, background.toVector());
    vectorToFile(inpainting, "images/inpaint.png", rows, cols);

    Retexture retextureObj;
    retextureObj.m_s = materialParams.s;

    std::vector<Vector3f> retexturing;
    retextureObj.calculateTexture(texture.toVector(), inpainting, im.toVector(), gradientX, gradientY, retexturing, mask );
    vectorToFile(retexturing, "images/output.png", rows, cols);
    return true;
}

bool MaterialManager::makeGlass(){
    if(!areBasicParamsValid()){
        return false;
    }
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    if(materialParams.backgroundFile == ""){
        // Use input image as background rather than replacing background.
        materialParams.backgroundFile = materialParams.mainImageFile;
    }
    ImageReader background(materialParams.backgroundFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;

    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);
    incidentlight incidentObj;
    std::vector<Vector3f> inpainting = incidentObj.inPaint(mask, background.toVector());
    vectorToFile(inpainting, "images/inpaint.png", rows, cols);

    Retexture retextureObj;

    if(!areGlassParametersValid()){
        return false;
    }

    retextureObj.m_frosty = materialParams.frosty;
    retextureObj.m_s = materialParams.s;
    retextureObj.m_glassColor = materialParams.glassColor;
    retextureObj.m_darkness = materialParams.darkness;

    std::vector<Vector3f> blurred = retextureObj.applyGaussianFilter(inpainting, cols, rows, retextureObj.m_frosty);
    vectorToFile(blurred, "images/inpaintBlur.png", rows, cols);

    std::vector<Vector3f> retexturing;
    retextureObj.calculate(blurred, inpainting, im.toVector(), gradientX, gradientY, retexturing, mask);

    materialResults.image = retexturing;
    materialResults.mask = mask.toVector();
    materialResults.normals = normals;
    materialResults.rows = rows;
    materialResults.cols = cols;

    BrdfReplacement br;
    br.replaceBrdf(retexturing, mask.toVector(), normals, rows, cols);
    materialResults.specularDirs = br.specularDirs;

    Histogram hist(se.getLuminances());
    std::vector<int> highlights = hist.findHighlights();

    std::vector<Vector3f> originalImage = im.toVector();
    \
    for (int i = 0; i < highlights.size(); i++) {
        int index = highlights[i];
        Vector3f originalVal = originalImage[index];
        int gray = (int(originalVal[0]) + int(originalVal[1]) + int(originalVal[2]))/3;
        retexturing[index] = Vector3f(gray, gray, gray);
    }
    vectorToFile(retexturing, "images/output.png", rows, cols);
    std::cout << "after vector to file" << std::endl;

    return true;
}

bool MaterialManager::makeCaustic(){
    // caustic
    if(!areBasicParamsValid()){
        return false;
    }
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    if(materialParams.backgroundFile == ""){
        // Use input image as background rather than replacing background.
        materialParams.backgroundFile = materialParams.mainImageFile;
    }
    ImageReader background(materialParams.backgroundFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;

    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);
    incidentlight incidentObj;
    std::vector<Vector3f> inpainting = incidentObj.inPaint(mask, background.toVector());
    vectorToFile(inpainting, "images/inpaint.png", rows, cols);

    Retexture retextureObj;

    if(!areGlassParametersValid()){
        return false;
    }

    retextureObj.m_frosty = materialParams.frosty;
    retextureObj.m_s = materialParams.s;

    std::vector<Vector3f> blurred = retextureObj.applyGaussianFilter(inpainting, cols, rows, retextureObj.m_frosty);
    vectorToFile(blurred, "images/inpaintBlur.png", rows, cols);

    std::vector<Vector3f> retexturing;
    retextureObj.calculate(blurred, inpainting, im.toVector(), gradientX, gradientY, retexturing, mask);

    Histogram hist(se.getLuminances());
    std::vector<int> highlights = hist.findHighlights();
    std::vector<Vector3f> originalImage = im.toVector();

    for (int i = 0; i < highlights.size(); i++) {
        int index = highlights[i];
        Vector3f originalVal = originalImage[index];
        int gray = (int(originalVal[0]) + int(originalVal[1]) + int(originalVal[2]))/3;
        retexturing[index] = Vector3f(gray, gray, gray);
    }
    vectorToFile(retexturing, "images/glass.png", rows, cols);



    std::cout << "creating python environment" << std::endl;
    QProcess p;
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("PYTHONPATH", "/Users/purvigoel/anaconda3/lib/python3.6/site-packages");
    QStringList params;
    params << "target_caustic_inverse.py" << materialParams.mainImageFile << materialParams.maskFile <<  ">>" << "log_caustic.txt";
    p.setStandardOutputFile("log.txt");
    p.start("/Users/purvigoel/anaconda3/bin/python", params);
    p.waitForFinished(-1);

    std::cout << "Solved caustic" << std::endl;

    //ImageReader c("images/caustic.png");
    std::ifstream infile("caustic_data.txt");
    std::string str;
    std::vector<Vector3f> c;
    while (std::getline(infile, str))
    {
        c.push_back(Vector3f(std::stof(str),std::stof(str),std::stof(str)));
    }

    CausticMaker cm(c, retexturing, rows, cols);
    std::vector<Vector3f> caustic = cm.project(0,0,50,0,50,50,0,50);

    vectorToFile(caustic, "images/output.png", rows, cols);
    return true;
}

bool MaterialManager::makeGlossy(int specular)
{
    std::cout<<"Before estimate shape"<<std::endl;
    ImageReader im(materialParams.mainImageFile);
    ImageReader mask(materialParams.maskFile);

    int cols = im.getImageWidth();
    int rows = im.getImageHeight();

    ShapeEstimation se;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    std::vector<float> depth;
    std::vector<Eigen::Vector3f> normals;
    std::vector<float> gradientX;
    std::vector<float> gradientY;

    if(!areShapeEstimationParamsValid()){
        return false;
    }

    se.m_bilateralSmoothing = materialParams.bilateralSmoothing;
    se.m_curvature = materialParams.curvature;


    se.estimateShape(im,mask, depth, normals, gradientX, gradientY);
    std::cout<<"Before get luminances"<<std::endl;
    Histogram hist(se.getLuminances());
    std::vector<float> highlights = hist.getHighlightsMaxAndMin();
    float min = highlights[0]*100.f;
    std::cout<< "Min highlights is: " << min <<std::endl;
    float max = highlights[1]*100.f;
    std::cout<< "Max highlights is: " << max << std::endl;
    float hMax = hist.findPeakHistogramValue();

    std::vector<Vector3f> originalImage = im.toVector();
    float alpha = 0.5;
    float beta = 20.f;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (QColor(mask.pixelAt(i, j)).red() > 150) {
                // In the object
                QColor pix = QColor(im.pixelAt(i, j));
                int index = im.indexAt(i, j);
                float lum = pix.red()*0.2126 + pix.green()*0.7152 + pix.blue()*0.0722;

                if (lum > min) {
                    // Calculate new luminance
                    if (specular) {
                        // Make more specular
                        beta = 20.f;
                        float partial = pow(alpha * ((lum - min) / (max - min)), beta);
                        float newlum = min + (max - min) * partial;
                        float ratio = (newlum/lum);
                        if (ratio > 1.3) {
                            ratio = 1.3f;
                        }
                        if (newlum > lum) {
                            std::cout<<"Ratio is: " << ratio << std::endl;
                        }

                        originalImage[index] = ratio*originalImage[index];
                    } else {
                        // Make more diffuse
                        beta = 0.05;
                        float partial = pow(((lum - hMax) / (max - hMax)), beta);
                        float newlum = hMax + (min - hMax) * partial;
                        originalImage[index] = ((1+(newlum/lum))/2.f)*originalImage[index];
                    }

                }
            }
        }
    }

    vectorToFile(originalImage, "images/output.png", rows, cols);
    std::cout << "after vector to file" << std::endl;

    return true;


}
