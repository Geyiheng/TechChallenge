﻿#include "display.h"
#include "interaction.h"
#include "visionmodule.h"
#include "parammanager.h"
#include "globaldata.h"
#include <limits>
#include "staticparams.h"
namespace{
const QColor COLOR_BACKGROUND(40,40,40);
const QColor COLOR_AXES(100,100,100);
auto opm = Owl::OParamManager::Instance();
template<typename T>
T limitRange(T value, T minValue, T maxValue) {
    return value > maxValue ? maxValue : (value < minValue) ? minValue : value;
}
float MAP_WIDTH;
float MAP_HEIGHT;
float MAXIMUM;
float x(float a){return limitRange(a,0.0f,1.0f)*MAP_WIDTH;};
float y(float a){return (1 - limitRange(a,0.0f,MAXIMUM)/MAXIMUM)*MAP_HEIGHT;};
float w(float a){return a*MAP_WIDTH;};
float h(float a){return -a/MAXIMUM*MAP_HEIGHT;};
}
Display::Display(QQuickItem *parent)
    : QQuickPaintedItem (parent)
    , pixmap(nullptr){
    setImplicitWidth(200);
    setImplicitHeight(300);
    pixmap = new QPixmap(QSize(200, 300));
    pixmapPainter.begin(pixmap);
    pixmapPainter.setPen(Qt::NoPen);
    pixmapPainter.setRenderHint(QPainter::HighQualityAntialiasing, true);
    pixmapPainter.setRenderHint(QPainter::TextAntialiasing, true);
    initAxes();
}
void Display::paint(QPainter* painter) {
    painter->drawPixmap(area, *pixmap);
}
void Display::draw() {
    repaint();
}
void Display::init(){
    initAxes();
    repaint();
}
void Display::initAxes(){
    if(opm->display_mode!="Vxy" && opm->display_mode!="BallSpeed")
        MAXIMUM = opm->maximum * 2;
    else
        MAXIMUM = opm->maximum;
    //MAP_WIDTH = this->property("width").toReal();
    //MAP_HEIGHT = this->property("height").toReal();
//    axesPath = QPainterPath();
//    QFont font("Helvetica [Cronyx]");
//    font.setWeight(QFont::ExtraLight);
//    font.setPixelSize(h(-0.5));
//    for(int i=0;i<displayColumn;i++){
//        axesPath.moveTo(x(0),y(maxSpeed/displayColumn*i));
//        axesPath.lineTo(x(1),y(maxSpeed/displayColumn*i));
//        axesPath.addText(QPointF(x(0),y(maxSpeed/displayColumn*i)),font,QString::number(maxSpeed/displayColumn*i,'g',2));
//    }
}
void Display::repaint(){
    if(repaint_mutex.tryLock()){
        pixmap->fill(COLOR_BACKGROUND);
        paintAxes();
        paintData();
        this->update(area);
        repaint_mutex.unlock();
    }
}
void Display::paintAxes(){
//    pen.setColor(COLOR_AXES);
//    pen.setWidth(1);
//    pixmapPainter.setPen(pen);
//    pixmapPainter.setBrush(Qt::red);
//    pixmapPainter.strokePath(axesPath, pen);
    pen.setColor(COLOR_AXES);
    pen.setWidth(1);
    pixmapPainter.setPen(pen);
    pixmapPainter.setFont(QFont("Helvetica [Cronyx]"));
    if(opm->display_mode=="Vxy" || opm->display_mode=="BallSpeed"){
        for(int i=0;i<=opm->horizontLines;i++){
            auto height = MAXIMUM/opm->horizontLines*i;
            pixmapPainter.drawLine(::x(0),::y(height),::x(1),::y(height));
            pixmapPainter.drawText(QPointF(::x(0),::y(height+0.1)),QString::number(height,'g',2));
        }
    }
    else {
        for(int i=0;i<=opm->horizontLines;i++){
            auto height = MAXIMUM/opm->horizontLines*i;
            pixmapPainter.drawLine(::x(0),::y(height),::x(1),::y(height));
            pixmapPainter.drawText(QPointF(::x(0),::y(height+0.1)),QString::number((height-MAXIMUM/2),'g',2));
        }
    }
    auto height = opm->limitation;
    pixmapPainter.setPen(Qt::red);
    if(opm->display_mode=="Vxy" || opm->display_mode=="BallSpeed") {
        pixmapPainter.drawLine(::x(0),::y(height),::x(1),::y(height));
        pixmapPainter.drawText(QPointF(::x(0),::y(height+0.1)),QString::number(height,'g',2));
    }
    else {
        pixmapPainter.drawLine(::x(0),::y(MAXIMUM/2+height),::x(1),::y(MAXIMUM/2+height));
        pixmapPainter.drawLine(::x(0),::y(MAXIMUM/2-height),::x(1),::y(MAXIMUM/2-height));
        pixmapPainter.drawText(QPointF(::x(0),::y(MAXIMUM/2+height+0.1)),QString::number(height,'g',2));
        pixmapPainter.drawText(QPointF(::x(0),::y(MAXIMUM/2-height+0.1)),QString::number(-height,'g',2));
    }
}
void Display::paintData(){
    QPoint lastPoint_m(::x(0.0f),::y(0.0f));
    QPoint lastPoint_c(::x(0.0f),::y(0.0f));
    QPoint lastPoint_r(::x(0.0f),::y(0.0f));
    QPoint lastPoint_a(::x(0.0f),::y(0.0f));
    QPoint newPoint_m = lastPoint_m;
    QPoint newPoint_c = lastPoint_c;
    QPoint newPoint_r = lastPoint_r;
    double maintain;
    if(opm->display_mode=="Vxy"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto command = GlobalData::Instance()->robotCommand[opm->display_teamIndex][i-Vision::MAINTAIN_STORE_BUFFER].robotSpeed[opm->display_robotID].vxy.mod();
            auto raw = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].raw.vel.vxy.mod();
            if (gd.robotIndex[opm->display_teamIndex][opm->display_robotID] == -1) maintain = 0;
            else maintain = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].velocity.vxy.mod();
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(maintain/1000.0f));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_c.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_c.setY(::y(command/1000.0f));
            pixmapPainter.setPen(Qt::yellow);
            pixmapPainter.drawLine(lastPoint_c,newPoint_c);
            lastPoint_c = newPoint_c;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(raw/1000.0f));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
    else if(opm->display_mode=="BallSpeed"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto raw = gd.ball->raw.vel.vxy.mod();
            maintain = gd.ball->velocity.mod();
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(maintain/1000.0f));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(raw/1000.0f));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
    else if(opm->display_mode=="Vx"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto command = GlobalData::Instance()->robotCommand[opm->display_teamIndex][i-Vision::MAINTAIN_STORE_BUFFER].robotSpeed[opm->display_robotID].vx();
            auto raw = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].raw.vel.vx();
            if (gd.robotIndex[opm->display_teamIndex][opm->display_robotID] == -1) maintain = 0;
            else maintain = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].velocity.vx();
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(MAXIMUM/2+maintain/1000.0f));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_c.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_c.setY(::y(MAXIMUM/2+command/1000.0f));
            pixmapPainter.setPen(Qt::yellow);
            pixmapPainter.drawLine(lastPoint_c,newPoint_c);
            lastPoint_c = newPoint_c;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(MAXIMUM/2+raw/1000.0f));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
    else if(opm->display_mode=="Vy"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto command = GlobalData::Instance()->robotCommand[opm->display_teamIndex][i-Vision::MAINTAIN_STORE_BUFFER].robotSpeed[opm->display_robotID].vy();
            auto raw = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].raw.vel.vy();
            if (gd.robotIndex[opm->display_teamIndex][opm->display_robotID] == -1) maintain = 0;
            else maintain = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].velocity.vy();
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(MAXIMUM/2+maintain/1000.0f));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_c.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_c.setY(::y(MAXIMUM/2+command/1000.0f));
            pixmapPainter.setPen(Qt::yellow);
            pixmapPainter.drawLine(lastPoint_c,newPoint_c);
            lastPoint_c = newPoint_c;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(MAXIMUM/2+raw/1000.0f));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
    else if(opm->display_mode=="Vr"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto command = GlobalData::Instance()->robotCommand[opm->display_teamIndex][i-Vision::MAINTAIN_STORE_BUFFER].robotSpeed[opm->display_robotID].vr;
            auto raw = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].raw.vel.vr;
            if (gd.robotIndex[opm->display_teamIndex][opm->display_robotID] == -1) maintain = 0;
            else maintain = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].velocity.vr;
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(MAXIMUM/2+maintain));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_c.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_c.setY(::y(MAXIMUM/2+command));
            pixmapPainter.setPen(Qt::yellow);
            pixmapPainter.drawLine(lastPoint_c,newPoint_c);
            lastPoint_c = newPoint_c;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(MAXIMUM/2+raw));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
    else if(opm->display_mode=="Angle"){
        for(int i=0;i<Vision::MAINTAIN_STORE_BUFFER;i+=1){
            auto gd = GlobalData::Instance()->maintain[1+i-Vision::MAINTAIN_STORE_BUFFER];
            auto raw = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].raw.angle;
            if (gd.robotIndex[opm->display_teamIndex][opm->display_robotID] == -1) maintain = 0;
            else maintain = gd.robot[opm->display_teamIndex][gd.robotIndex[opm->display_teamIndex][opm->display_robotID]].angle;
            newPoint_m.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_m.setY(::y(MAXIMUM/2+maintain));
            pixmapPainter.setPen(QColor(200,200,200));
            pixmapPainter.drawLine(lastPoint_m,newPoint_m);
            lastPoint_m = newPoint_m;

            newPoint_r.setX(::x((float)i/Vision::MAINTAIN_STORE_BUFFER));
            newPoint_r.setY(::y(MAXIMUM/2+raw));
            pixmapPainter.setPen(Qt::green);
            pixmapPainter.drawLine(lastPoint_r,newPoint_r);
            lastPoint_r = newPoint_r;
        }
    }
}
void Display::resetSize(int width,int height){
    pixmapPainter.end();
    delete pixmap;
    pixmap = new QPixmap(QSize(this->property("width").toReal(), this->property("height").toReal()));
    pixmapPainter.begin(pixmap);
    MAP_WIDTH = this->property("width").toReal();
    MAP_HEIGHT = this->property("height").toReal();
    area = QRect(0, 0, this->property("width").toReal(), this->property("height").toReal());
    init();
}

void Display::ifNeedDisplay(bool needDisplay){
    if(needDisplay) {
        connect(VisionModule::Instance(), SIGNAL(needDraw()), this, SLOT(draw()));
    }
    else {
        disconnect(VisionModule::Instance(), SIGNAL(needDraw()), this, SLOT(draw()));
    }
}

void Display::setDisplayMode() {
    QDialog dialog;
    QFormLayout form(&dialog);
    form.addRow(new QLabel("Set robot:"));
    QString value1 = QString("team: ");
    QComboBox *comboBox1 = new QComboBox(&dialog);
    comboBox1->addItem("Blue");
    comboBox1->addItem("Yellow");
    comboBox1->setCurrentIndex(opm->display_teamIndex);
    form.addRow(value1, comboBox1);
    QString value2 = QString("robotID: ");
    QSpinBox *spinbox2 = new QSpinBox(&dialog);
    spinbox2->setRange(0, PARAM::ROBOTMAXID);
    spinbox2->setValue(opm->display_robotID);
    form.addRow(value2, spinbox2);
    form.addRow(new QLabel("Set Display Mode:"));
    QComboBox *comboBox3 = new QComboBox(&dialog);
    comboBox3->addItem("Vxy");
    comboBox3->addItem("Vx");
    comboBox3->addItem("Vy");
    comboBox3->addItem("Vr");
    comboBox3->addItem("Angle");
    comboBox3->addItem("BallSpeed");
    comboBox3->setCurrentIndex(opm->display_modeIndex);
    form.addRow(comboBox3);
    form.addRow(new QLabel("Set Feature:"));
    QString value7 = QString("height: ");
    QSpinBox *spinbox7 = new QSpinBox(&dialog);
    spinbox7->setRange(0, 2000);
    spinbox7->setValue(opm->display_height);
    form.addRow(value7, spinbox7);
    QString value4 = QString("horizontLines: ");
    QSpinBox *spinbox4 = new QSpinBox(&dialog);
    spinbox4->setRange(0, 1000);
    spinbox4->setValue(opm->horizontLines);
    form.addRow(value4, spinbox4);
    QString value5 = QString("maximum: ");
    QDoubleSpinBox *spinbox5 = new QDoubleSpinBox(&dialog);
    spinbox5->setRange(0, 1000);
    spinbox5->setValue(opm->maximum);
    form.addRow(value5, spinbox5);
    QString value6 = QString("limitation: ");
    QDoubleSpinBox *spinbox6 = new QDoubleSpinBox(&dialog);
    spinbox6->setRange(0, 1000);
    spinbox6->setValue(opm->limitation);
    form.addRow(value6, spinbox6);
    /**QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 130);
    slider->setValue(opm->limitation);
    QObject::connect(spinbox6, SIGNAL(valueChanged(int)),slider, SLOT(setValue(int)));
    QObject::connect(slider, SIGNAL(valueChanged(int)),spinbox6, SLOT(setValue(int)));
    form.addRow(slider);**/
    // Add Cancel and OK button
    QDialogButtonBox buttonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
    form.addRow(&buttonBox);
    QObject::connect(&buttonBox, SIGNAL(accepted()), &dialog, SLOT(accept()));
    QObject::connect(&buttonBox, SIGNAL(rejected()), &dialog, SLOT(reject()));
    if (dialog.exec() == QDialog::Accepted) {
        opm->updateParam(opm->display_team, "Display/display_team", comboBox1->currentText(), true);
        opm->updateParam(opm->display_teamIndex, "Display/display_teamIndex", comboBox1->currentIndex(), true);
        opm->updateParam(opm->display_robotID, "Display/display_robotID", spinbox2->text().toInt(), true);
        opm->updateParam(opm->display_mode, "Display/display_mode", comboBox3->currentText(), true);
        opm->updateParam(opm->display_modeIndex, "Display/display_modeIndex", comboBox3->currentIndex(), true);
        opm->updateParam(opm->display_height, "Display/display_height", spinbox7->text().toDouble(), true);
        opm->updateParam(opm->horizontLines, "Display/horizontLines", spinbox4->text().toInt(), true);
        opm->updateParam(opm->maximum, "Display/maximum", spinbox5->text().toDouble(), true);
        opm->updateParam(opm->limitation, "Display/limitation", spinbox6->text().toDouble(), true);
    }
    else return;
    init();
}

int Display::getDisplayHeight() {
    return opm->display_height;
}

