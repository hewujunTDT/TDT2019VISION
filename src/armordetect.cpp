#include "armordetect.h"
#include "debug.h"
#include "tnet.h"
using std::vector;
using namespace cv::ml;
using namespace cv;
using namespace std;
////////////////////////////// ArmorDetect //////////////////////////////
ArmorDetector::ArmorDetector(const Camera &camera){
    enemy_color_=Color(Parameter.enemy_colour);
    offset_=Point(0,0);  //初始化偏移量
    same_armor_= false;
}

ArmorDetector::~ArmorDetector() = default;

vector<Armor> ArmorDetector::Get(Mat &src,ReciveMessage recive_message ){
    RoiFilter(src, offset_);//提取ROI区域

#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.src=new Mat();
        src.copyTo(*G_armorinformation.src);
        G_armorinformation.roi_area=roi_rect_;
    }
#endif
    vector<Armor> Armors;
    LightbarDetector _Detecter(enemy_color_, offset_,roi_rect_,last_armor_);

    _Detecter.Detect(src, Armors);  //输出所有认为是装甲板的Armor
    same_armor_ = false;
    for(auto & Armor : Armors){
        if(Armor.get_robot_type() == last_armor_.get_robot_type()) {
            same_armor_ = true; break;
        }
    }
//    上一帧识别到装甲板 && 丢失上一帧目标
    if (!same_armor_ &&!last_armor_.empty()) {
        cout << "------基于数字检测装甲板------" << endl;
        NumberStikerDetector _Num_Detecter(enemy_color_, offset_, roi_rect_, last_armor_);  //无灯条装甲板识别
        vector<Armor> no_lightbar_armors;
        _Num_Detecter.Detect(src, no_lightbar_armors);
        Armors.insert(Armors.end(), no_lightbar_armors.begin(), no_lightbar_armors.end());
    }
    RobotType type=Decide(Armors,recive_message);
    vector<Armor> results;
    for(auto & Armor : Armors) {
        if(Armor.get_robot_type()==type){
            results.push_back(Armor);
        }
    }

    if(results.size()>1){//剔除一下两个装甲板重叠的情况
        vector<Armor> results_tmp;
        for(int i = 0; i<results.size();i++) {
            int judge=1;
            for(int j = 0; j<results.size();j++) {
                if(j==i)continue;
                Rect a=results[i].get_numberstiker().get_bounding_rect();
                Rect b=results[j].get_numberstiker().get_bounding_rect();
                Rect c=a&b;
                if(c.size().area()>0){
                    if(getDistance(results[i].get_left_lightbar().get_center(),results[i].get_right_lightbar().get_center())>
                       getDistance(results[j].get_left_lightbar().get_center(),results[j].get_right_lightbar().get_center())){
                        judge=0;
                    }
                }
            }
            if(judge){
                results_tmp.push_back(results[i]);
            }
        }
        results_tmp.swap(results);

    }
    G_armorinformation.tmpparam0=results.size();

        if(!results.empty()){
        last_armor_=results[0];
    }
    vector<Armor>().swap(Armors);  //释放vector

    return results;
}

RobotType ArmorDetector::Decide(vector<Armor> &armors,ReciveMessage &recive_message){
    RobotType ret=TYPEUNKNOW;
    if(recive_message.lock_command&&!last_armor_.empty()){
        ret=last_armor_.get_robot_type();
    }else {
        sort(armors.begin(),armors.end(),
             [](Armor a,Armor b)->
                     bool{ return a.get_distance_to_screen_center()<b.get_distance_to_screen_center();});
        for(int i = 0; i < armors.size(); i++){
            if(armors[i].get_robot_type() == ENGINEER &&i!=armors.size()-1) continue ;
            i=(i==armors.size()-1) ? 0:i;
            ret=armors[i].get_robot_type();
            break;
        }
    }
    return ret;
}

void ArmorDetector::RoiFilter(Mat &input, Point &offset) {
    if(!last_armor_.empty()){
        roi_rect_=last_armor_.get_rect();
        Size enlarge_size =last_armor_.get_armor_type() == 1?Size(7,5):Size(9,7);
        if(last_armor_.get_form()[0] != last_armor_.get_form()[1])
            enlarge_size = Size(14,5);
        roi_rect_=rectCenterScale(roi_rect_,enlarge_size);
        if (!RectSafety(roi_rect_, input.rows, input.cols)){
            roi_rect_=Rect(Point(0,0),input.size());
        }
    }else{
        roi_rect_=Rect(Point(0,0),input.size());
    }
    roi_rect_=Rect(Point(0,0),input.size());

    offset = roi_rect_.tl();      //防止区域扣出原图外，并把ROI区域的左上角点作为补偿的坐标点

}

////////////////////////////// Detecter //////////////////////////////
Detector::Detector(Color color,Point offset_){
    enemy_color_=color;
    offset=offset_;
}

void Detector::TnetDL(vector<Armor> &input_armors) {


    vector<Mat> ml_rois;
    vector<int> results;
#ifdef USEMXNET
    for(Armor &armor :input_armors){
        Mat armorimg=armor.get_ml_roi();
        armorimg.convertTo(armorimg,CV_32F);
        normalize(armorimg,armorimg);
        ml_rois.push_back(armorimg);
    }
    static Tnet tdtnet(0);
    tdtnet.predict(ml_rois,results);
    for(int i=0; i<input_armors.size(); i++) {
        if(input_armors[i].get_robot_type()==SENTRY) continue;
        if(results[i]==7){
            input_armors[i].set_robot_type(SENTRY);
        } else if(results[i]==8){
            input_armors[i].set_robot_type(BASE);
        }else
            input_armors[i].set_robot_type(RobotType(results[i]));
    }
#endif
#ifdef USESVM
    static HOGDescriptor HogDescriptor(Size(24, 24),Size(8, 8),Size(4, 4),Size(4, 4),9);
    static Ptr<cv::ml::SVM> svm_hog = cv::ml::SVM::load("../model/SVM(NEW_SMALL).xml");//载入机器学习模型
    for(Armor &armor :input_armors){
        if(armor.get_robot_type()==SENTRY) continue;
        Mat armorimg=armor.get_ml_roi();
        resize(armorimg,armorimg,Size(24,24));
        vector<float> ArmorHog;
        HogDescriptor.compute(armorimg, ArmorHog);
        Mat armorpredict(ArmorHog,CV_32F);
        transpose(armorpredict,armorpredict);
        results.push_back(svm_hog->predict(armorpredict));
    }
    for(int i=0; i<input_armors.size(); i++) {
        if( input_armors[i].get_robot_type()==SENTRY) continue;
        input_armors[i].set_robot_type(RobotType(results[i]));
    }
#endif
}

void Detector::SetMlRoi(Armor &armor,const Mat &src) {
    Mat armorimg;
    RotatedRect roi=armor.get_armor_rotatedrect();
    Rect roi_mat=roi.boundingRect();
    Rect roi_matcopy=Rect(roi_mat);
    if(RectSafety(roi_matcopy,src.size())){
        Mat roiimg;
        if(roi_matcopy!=roi_mat){
            roiimg=Mat::zeros(roi_mat.size(),CV_8UC3);
            src(roi_matcopy).copyTo(roiimg(Rect(roi_matcopy.tl()-roi_mat.tl(),roi_matcopy.size())));
        } else{
            roiimg=src(roi_mat);
        }
        float angle=armor.get_numberstiker().get_angle();

        Mat rotationmat = getRotationMatrix2D(Point(roi.center)-roi_mat.tl(),angle, 1);      //仿射变换
        warpAffine(roiimg,armorimg,rotationmat,roiimg.size()); //TODO 会把灯条框进去
        Size size_;
        if(roi.angle>=-45){
            size_=roi.size;
        } else{
            size_=Size(cvRound(roi.size.height),cvRound(roi.size.width));
        }
        Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
        if(RectSafety(realsizerect,armorimg.size()))
        {  armorimg = armorimg(realsizerect);
        } else{
            armorimg = Mat::zeros(28,28,CV_8UC3);
        }
    } else{
        armorimg = Mat::zeros(28,28,CV_8UC3);
    }

    Mat rgb[3];
    split(armorimg,rgb);
//    armorimg = rgb[2-enemy_color_];
    cvtColor(armorimg,armorimg,CV_BGR2GRAY);

    resize(armorimg,armorimg,Size(28,28));
    armor.set_ml_roi(armorimg);
}

float Detector::FastTargetLock(Armor &armor,Armor last_armor) {
    if(last_armor.empty()) return false;
    RotatedRect r1=armor.get_numberstiker().get_rotated_rect();
    RotatedRect r2=last_armor.get_numberstiker().get_rotated_rect();
    float area_deviation = 0.2f*fabs(r1.size.area()-r2.size.area())/r2.size.area();
    float dist_deviation = 0.1f*getDistance(r1.center,r2.center)/r2.size.height;
    if(area_deviation+dist_deviation>0.2) return 0;

    static HOGDescriptor *hog = new HOGDescriptor(Size(28, 28), Size(14, 14), Size(7, 7), Size(7, 7), 9);
    vector<float> descriptors;
    hog->compute(armor.get_ml_roi(), descriptors,Size(1, 1), Size(0, 0));
    Mat des=Mat(descriptors);
    transpose(des,des);
    armor.set_ml_hog(des);//保存hog矩阵属性

    static 	Ptr<KNearest> KnnModel = StatModel::load<KNearest>("../model/knn(7).xml");
    Mat result,nearest,dist;
    KnnModel->findNearest(armor.get_ml_hog(),1,result,nearest,dist);
    if(dist.at<float>(0,0)<1.8){
        armor.set_robot_type(SENTRY);
    }
    if(last_armor.get_ml_hog().empty()){
        hog->compute(last_armor.get_ml_roi(), descriptors,Size(1, 1), Size(0, 0));
        des=Mat(descriptors);
        transpose(des,des);
        last_armor.set_ml_hog(des);
    }


    float  similarity=getSimilarity(armor.get_ml_hog(),last_armor.get_ml_hog());
    similarity-=(area_deviation+dist_deviation);
    return 0.3;
}

//////////////////////////////LightbarDetector///////////////////////////////

void LightbarDetector:: Detect(Mat &src,vector<Armor> &output_armors){
    vector<LightBar> lightbars;

    ArmorDetector_GetLightBar(src,lightbars);

    ArmorDetector_GetArmor(src,lightbars,output_armors);
}

void LightbarDetector::ArmorDetector_GetLightBar(Mat &src,vector<LightBar> &lightbars) {
    Mat gray_thresholded; //二值化图
    Mat roi;              //感兴趣区域
    roi = src(roi_area_);

    Mat img_gray_1,img_gray_2;  //灰度图
    Mat img_gray;
    vector<Mat> rgb;
    split(roi,rgb);
    img_gray_1=rgb[enemy_color_]-rgb[2-enemy_color_];
    cvtColor(roi,img_gray_2,COLOR_BGR2GRAY);  //转灰度图
    img_gray=img_gray_1 | img_gray_2;

#ifdef DEBUG
    if(G_setval.WORK) {
        Parameter.ArmorDetect_lightbarthre = *G_setval.lightbar_threval;
    }
#endif
    threshold(img_gray,gray_thresholded,Parameter.ArmorDetect_lightbarthre,255,THRESH_BINARY);//将灰度图二值化 寻找灯条解


#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.lightbar_thre=new Mat(gray_thresholded.clone());
    }
#endif

    vector< vector< cv::Point> > contours;  //轮廓容器
    findContours(gray_thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,offset); //寻找轮廓

    for(vector<Point> const &contour : contours){ //遍历所有轮廓
        double area =contourArea(contour, false);
        if(area< 10 || area>12000) continue;//面积太小太小的不要
        RotatedRect rotated_rect=minAreaRect(contour); //最小包围旋转矩形
        LightBar lightbar(rotated_rect,Color(UNKNOW));  //利用旋转矩形封装Linghtbar
        if(lightbar.get_ratio()>0.8) {  //根据长宽比和角度筛选灯条
            continue;
        }
        if(!JudgeColor(lightbar,src)) continue;
        lightbars.push_back(lightbar);
    }
    sort(lightbars.begin(),lightbars.end(),
         [](LightBar a,LightBar b)->bool{ return a.get_center().x < b.get_center().x;});

}

void  LightbarDetector::ArmorDetector_GetArmor(const Mat &src, vector<LightBar> &lightbars,vector<Armor> &output_armors){
    ////--------双灯条封装Armor---------
    for(int i=0; i<lightbars.size(); i++)//双层循环 遍历每一对矩形框
    {
        for(int j=i+1; j < lightbars.size(); j++) {
            if (!GetEligibility(lightbars[i],lightbars[j])) continue;//合格性判断
            Rect search_rect=get_search_rect(lightbars[i],lightbars[j]); //左右灯条中间区域
            if(!RectSafety(search_rect, src.size())) continue;   //安全性
            Mat image_numb=src(search_rect);
            vector<Mat> rgb;
            int thre=0;
            cvtColor(image_numb,image_numb,CV_BGR2GRAY);

            if(!region_otsu_threshold(image_numb,image_numb,thre, 0)) continue;
            
            vector<vector<Point>>contours;
            findContours(image_numb,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,search_rect.tl());
            vector<RotatedRect> Numb_rects;
            LightBar lightbar=lightbars[i].get_area()>lightbars[j].get_area()?lightbars[i]:lightbars[j];
            for(vector<Point> const &contour : contours) {
                if (contour.size()<20 ) continue;//面积太小的不要
                RotatedRect Numb_rect = minAreaRect(contour,(int)lightbar.get_rotatedrect().angle,0);
                Rect brect=Numb_rect.boundingRect();
                if(brect.width>brect.height)continue;

                if (Numb_rect.size.area() < 0.1* search_rect.area()) continue;

                float dist1=getDistance(Point(Numb_rect.center),lightbars[i].get_center());
                float dist2=getDistance(Point(Numb_rect.center),lightbars[j].get_center());
                if(lightbars[i].get_height()>lightbars[j].get_height()){
                    if(dist1>1.7*dist2||dist2>1.3*dist1)continue;
                } else{
                    if(dist1>1.3*dist2||dist2>1.7*dist1)continue;
                }

                Numb_rects.push_back(Numb_rect);
            }

            NumberStiker numberStiker;
            if(Numb_rects.empty()){
                continue;
            } else {
                sort(Numb_rects.begin(),Numb_rects.end(),
                     [](RotatedRect a,RotatedRect b)->bool{ return a.size.area()>b.size.area();});//以旋转矩形面积从大到小排序
                RotatedRect Numb_rotatedrect=Numb_rects[0];
                numberStiker=NumberStiker(Numb_rotatedrect);//将search封装为贴纸
                Rect numberbr=Numb_rotatedrect.boundingRect();
                if(!RectSafety(numberbr,src.size())) continue;
                numberStiker.set_bounding_rect(numberbr);
            }
            Armor tmp_armor=Armor(numberStiker,lightbars[i],lightbars[j]);//封装一个Armor
            tmp_armor.set_num_threshold(thre);
            lightbars[i].set_judge_search(lightbars[i].get_judge_search()==-1?2:1);//-1表示此灯条寻找过左边
            lightbars[j].set_judge_search(lightbars[j].get_judge_search()==1?2:-1);//1表示寻找过右边
            SetMlRoi(tmp_armor,src);//设置贴纸区域
            tmp_armor.set_similar_to_last(FastTargetLock(tmp_armor,last_Armor_));
            output_armors.push_back(tmp_armor);
            break;
        }
    }


////-------单灯条封装Armor函数-----------

    for(LightBar const &lightbar : lightbars) {
        if(lightbar.get_area()<200)continue;
        if(lightbar.get_judge_search()==2)continue;
        for (int lr=-1;lr<2;lr+=2){
            if(lr== lightbar.get_judge_search()) continue;
            RotatedRect search_rect = get_single_rotatedrect(lightbar, bool(lr==1));
            if(search_rect.boundingRect().area()<500) continue;
            Rect rectsaft=search_rect.boundingRect();
            if(!RectSafety(rectsaft, src.size()))continue;
            Mat last_image=src(rectsaft).clone();
            vector<Mat> rgb2;
            split(last_image,rgb2);
            last_image=rgb2[2-enemy_color_];
            int thre;
            if(!region_otsu_threshold(last_image,last_image,thre, lr)) continue;

            vector<vector<Point>>contours;
            findContours(last_image,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,rectsaft.tl());
            vector<RotatedRect> Numb_rects;
            for(vector<Point> const &contour : contours) {
                if (contour.size() < 60) continue;
                RotatedRect Numb_rect = minAreaRect(contour,lightbar.get_rotatedrect().angle,0);
                Rect brect=Numb_rect.boundingRect();
                if(brect.width>0.8*brect.height)continue;
                if (Numb_rect.size.area() < 0.05 * rectsaft.size().area())continue;
                if (Numb_rect.size.area() > 0.8 * rectsaft.size().area())continue;
                Numb_rects.push_back(Numb_rect);
            }

            if(Numb_rects.empty())continue;

            sort(Numb_rects.begin(),Numb_rects.end(),
                 [](RotatedRect a,RotatedRect b)->bool{ return a.size.area()>b.size.area();});//排序
            //选择靠近灯条位置的轮廓
            if(Numb_rects.size()>1){
                Numb_rects[0]=(Numb_rects[0].center.x<Numb_rects[1].center.x)==bool(lr==1)
                              ? Numb_rects[0]:Numb_rects[1];
            }
            NumberStiker numberstiker=NumberStiker(Numb_rects[0]);

            Rect numberbr=Numb_rects[0].boundingRect();
            if(!RectSafety(numberbr,src.size())) continue;
            numberstiker.set_bounding_rect(numberbr);
            Armor tmp_armor=Armor(numberstiker,lightbar);
            tmp_armor.set_num_threshold(thre);
            SetMlRoi(tmp_armor,src);
            tmp_armor.set_similar_to_last(FastTargetLock(tmp_armor,last_Armor_));

            output_armors.push_back(tmp_armor);
        }
    }
#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.lightbars=lightbars;
    }
#endif
    if(output_armors.empty())return;
    sort(output_armors.begin(),output_armors.end(),
         [](Armor a,Armor b)->
                 bool{return a.get_similar_to_last()> b.get_similar_to_last();});//相似度排序
    if(output_armors[0].get_similar_to_last()>0.8){
        output_armors[0].set_robot_type(last_Armor_.get_robot_type());
        output_armors={output_armors[0]};
    } else{
        TnetDL(output_armors);//深度学习判断数字
    }

#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.armors=output_armors;
    }
#endif
    vector<Armor> tmp_output_armors;
    for(Armor& armor :output_armors){
        if(armor.get_robot_type()!=TYPEUNKNOW){
            tmp_output_armors.push_back(armor);
        }
    }
    tmp_output_armors.swap(output_armors);
    tmp_output_armors.clear();
}

bool LightbarDetector::region_otsu_threshold(const Mat &inputimage, Mat &outputimage, int &thre, int lr) {
    bool ret= false;
    Mat sum_row,judge;
    Mat tmp;
    Rect orect;
    if(lr==-1){
        orect=Rect(Point(cvRound(0.5*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.9*inputimage.cols),cvRound(0.65*inputimage.rows)));
    } else if(lr==1){
        orect=Rect(Point(cvRound(0.1*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.5*inputimage.cols),cvRound(0.65*inputimage.rows)));
    } else{
        orect=Rect(Point(cvRound(0.3*inputimage.cols),cvRound(0.35*inputimage.rows)),
                   Point(cvRound(0.7*inputimage.cols),cvRound(0.65*inputimage.rows)));
    }
    thre=cvRound(threshold(inputimage(orect),tmp,0,255,THRESH_OTSU));
    threshold(inputimage,outputimage,thre,1,THRESH_BINARY);//将灰度图二值化 寻找灯条

    if(sum(outputimage)[0]>0.6*outputimage.size().area())
        return false;

    reduce(outputimage(Range(outputimage.rows*3/10,outputimage.rows*7/10),
                       Range::all()),sum_row,1, REDUCE_SUM,CV_32F);
    threshold(sum_row,judge,2,1,THRESH_BINARY_INV);
    if (sum(judge)[0] <= 1) {
        threshold(sum_row,judge,outputimage.cols*0.5,1,THRESH_BINARY);
        if (sum(judge)[0] < 0.5*sum_row.rows) {
            ret= true;
        }
    }
    outputimage*=255;
    return ret;
}

int LightbarDetector::JudgeColor(LightBar &lightbar,Mat &src) {  //判断灯条颜色

    bool flag= false;
    Rect judge_rect=lightbar.get_bounding_rect();
    RectSafety(judge_rect, src.rows, src.cols);
    Scalar jud=sum(src(judge_rect));
    if(jud[enemy_color_]>jud[2-enemy_color_]){
        flag= true;
    }
    return flag;
}

bool LightbarDetector:: GetEligibility(const LightBar &lightbar1, const LightBar &lightbar2) {
    LightBar left_lightbar=lightbar1;
    LightBar right_lightbar=lightbar2;
    LeftRightLightBarSafety(left_lightbar,right_lightbar);
    float tan_angle;//两个矩形框质心连线与水平线夹角的正切

    float left_lightbar_angle = left_lightbar.get_angle();
    float right_lightbar_angle = right_lightbar.get_angle();
    Point left_lightbar_center = left_lightbar.get_center();
    Point right_lightbar_center = right_lightbar.get_center();
    int length_min = min(left_lightbar.get_height(), right_lightbar.get_height());
    int length_max = max(left_lightbar.get_height(), right_lightbar.get_height());

    if (fabs(left_lightbar.get_center().y - right_lightbar.get_center().y) >
        length_max) {
        if (fabs(left_lightbar.get_center().x - right_lightbar.get_center().x) >
            length_max)
            return false;
    }

    if (fabs(left_lightbar_angle - right_lightbar_angle) > 15) {
        return false;
    }
    float point_distance = getDistance(left_lightbar_center, right_lightbar_center);
    if ( (point_distance > 9 *length_max)  || point_distance < length_min) { return false; }
    if (length_max / length_min > 2) { return false; }
    if (right_lightbar_center.x == left_lightbar_center.x) {//防止除0的尴尬情况
        tan_angle = 100;
    } else {
        tan_angle = atan(fabs((right_lightbar_center.y - left_lightbar_center.y) /
                              static_cast<float>((right_lightbar_center.x - left_lightbar_center.x))));
    }
    float grade = 200;//总分
    grade = grade - float((tan_angle) * 180 )-fabs(left_lightbar_angle - right_lightbar_angle);
    bool flag=grade>150;

    return flag;
}

Rect LightbarDetector::get_search_rect(const LightBar &lightbar1, const LightBar &lightbar2) {   //双灯条获得贴纸直矩形
    //根据LeftRightLightBarSafety灯条中点坐标区分左右灯条
    LightBar left_lightbar=lightbar1;
    LightBar right_lightbar=lightbar2;
    LeftRightLightBarSafety(left_lightbar,right_lightbar);
    Point left_lightbar_center=left_lightbar.get_center();
    Point right_lightbar_center=right_lightbar.get_center();

    int height_min = max(left_lightbar.get_height(),right_lightbar.get_height());
    Point search_rect_tl,search_rect_br;

    search_rect_tl.x=max(left_lightbar.br().x,left_lightbar.tr().x)+6;
    search_rect_tl.y= (left_lightbar_center.y+right_lightbar_center.y)/2 -
                      height_min-abs(left_lightbar_center.y-right_lightbar_center.y);
    search_rect_br.x=min(right_lightbar.bl().x,right_lightbar.tl().x)-6;
    search_rect_br.y=(left_lightbar_center.y+right_lightbar_center.y)/2 +
                     height_min+abs(left_lightbar_center.y-right_lightbar_center.y);

    Rect search_rect=Rect(search_rect_tl,search_rect_br);

    return search_rect;
}

RotatedRect LightbarDetector::get_single_rotatedrect(const LightBar &lightBar,bool right_or_left) { //right=1,left=0
    RotatedRect search_rotatedrect;
    Point rcentor=Point2f(lightBar.get_center().x+0.7f*lightBar.get_width()+ 1.f *lightBar.get_height(),
                          lightBar.get_center().y);
    Size rsize=Size2f(1.3f*lightBar.get_height(),2.3f*lightBar.get_height());
    if(!right_or_left) rcentor = 2*lightBar.get_center()- rcentor;
    search_rotatedrect=RotatedRect(rcentor,rsize,lightBar.get_angle());

    return search_rotatedrect;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void NumberStikerDetector::Detect(Mat &src,vector<Armor> &output_armors) {

    this->last_threshold_=last_armor_.get_num_threshold();  //获得颜色取反之后的大津法阈值
    Rect safetyrect;
    safetyrect=last_armor_.get_rect();
    safetyrect=rectCenterScale(safetyrect,Size2f(2,2));
    if(!RectSafety(safetyrect,src.size()))
        return;
    Mat src_roi = src(safetyrect);
    vector<vector<Point>> contour;
    vector<Mat> split_mat;
    split(src_roi,split_mat);
    src_roi=split_mat[2-enemy_color_];//消除灯条干扰0蓝2红
    threshold(src_roi,src_roi,max(20,last_threshold_),255,THRESH_BINARY);//根据上一帧装甲板计算出的阈值二值化图片
    findContours(src_roi,contour,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,safetyrect.tl());//提取外轮廓

    for(size_t i=0;i<contour.size();i++){
        if (contour[i].size() < 20) continue;//轮廓面积的绝对值
        RotatedRect rot_rect=minAreaRect(contour[i],last_armor_.get_armor_angle(),0);//旋转矩形框
        if(rot_rect.size.area()<last_armor_.get_numberstiker().get_rotated_rect().size.area()*0.5) continue;
        NumberStiker numtmp(rot_rect);
        Armor tmp(numtmp);
        tmp.set_armor_type(NoLightbar);
        SetMlRoi(tmp,src);
        tmp.set_similar_to_last(FastTargetLock(tmp,last_armor_));
        tmp.set_num_threshold( this->last_threshold_);
        output_armors.push_back(tmp);
    }


    if(output_armors.empty())return;
    sort(output_armors.begin(),output_armors.end(),
         [](Armor a,Armor b)->
                 bool{return a.get_similar_to_last()> b.get_similar_to_last();});
    if(output_armors[0].get_similar_to_last()>0.7){
        output_armors[0].set_robot_type(last_armor_.get_robot_type());
        output_armors={output_armors[0]};
    } else if(output_armors[0].get_similar_to_last() > 0.2){
        TnetDL(output_armors);
    }
    vector<Armor> tmp_output_armors;
    for(Armor& armor :output_armors){
        if(armor.get_robot_type() == last_armor_.get_robot_type()){
            tmp_output_armors.push_back(armor);
        }
    }

#ifdef DEBUG
    if(G_setval.WORK){
        G_armorinformation.armors.insert(G_armorinformation.armors.end(),tmp_output_armors.begin(),tmp_output_armors.end());
    }
#endif
    tmp_output_armors.swap(output_armors);
    tmp_output_armors.clear();
}
