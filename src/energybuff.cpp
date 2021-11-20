#include <energybuff.h>


vector<EnergyBuffArmor> EnergyBuffDetector::Get(Mat &src){
    vector<EnergyBuffArmor> armors;
    Detect(armors,src);//图像中识别所有大神符装甲
    vector<EnergyBuffArmor> ret_armors(5);//返回的vector,size为5
    sort(armors.begin(),armors.end(),
         [](EnergyBuffArmor a,EnergyBuffArmor b)->
                 bool{ return (a.get_kd_flowwater()+a.get_kd_r())
                 <(b.get_kd_flowwater()+b.get_kd_r());});
    G_buffinformation.tmpparam6 = armors.size();
    for(EnergyBuffArmor &armor:armors){
        if(armor.get_type()==FlowWater){//第0个是流水灯
            G_buffinformation.tmpparam4 = armor.get_kd_r();
            G_buffinformation.tmpparam5 = armor.get_kd_flowwater();
            if(armor.get_kd_r()+armor.get_kd_flowwater()<13.5) {
                ret_armors[0] = armor;
                break;
            } else{
                if (fabs(armor.get_rotatedrect().size.area()/last_armor_area-1)<0.1&&
//                     getDistance(armor.get_circle(),get_last_circle_center()) < 50 &&
                    fabs(get_last_radius()/getDistance(armor.center(),armor.get_circle())-1)<0.1){
                    ret_armors[0] = armor;
                    break;
                } else{
                    continue;
                }
            }
        }
    }

    set_last_circle_center(ret_armors[0].get_circle());
    set_last_radius(ret_armors[0].get_circle(),ret_armors[0].center());
    last_armor_area=ret_armors[0].get_rotatedrect().size.area();

    if(ret_armors[0].get_type()==FlowWater){//如果检测到了流水灯

        for(EnergyBuffArmor &armor:armors){
            if(armor.get_type()!=NoFlowWater)continue;//遍历已经激活过的装甲板
            if(armor.get_circle()!=ret_armors[0].get_circle())continue;
            int idex;
            float diffangle=armor.get_angle()-ret_armors[0].get_angle();//通过角度差按顺时针顺序排序
            if(diffangle>0){//获得真正的角度差
                idex=cvRound(diffangle/(2*kPI/5));
            } else{
                idex=5+cvRound(diffangle/(2*kPI/5));
            }
            if(idex<1||idex>4)continue;
            ret_armors[idex]=armor;
        }
    }
#ifdef DEBUG
    if(G_setval.WORK){
        G_buffinformation.buffarmors=ret_armors;
    }
#endif

    return ret_armors;
}

void EnergyBuffDetector::Detect(vector<EnergyBuffArmor> &armors,const Mat &src){
    Mat rgb_img[3];
    split(src, rgb_img);

    Mat color_img,gray_img,bin;
    subtract(rgb_img[my_color_], rgb_img[2-my_color_], color_img);//通道相减


    threshold(color_img, bin, Parameter.EnergyBuffDetect_split_thre, 255, THRESH_BINARY);

    cvtColor(src,gray_img,CV_BGR2GRAY);
    Mat gray_bin;
    threshold(gray_img, gray_bin, Parameter.EnergyBuffDetect_gray_thre, 255, THRESH_BINARY);
    bin=bin&gray_bin;



    if(Parameter.EnergyBuffDetect_dialesize!=0){
        int &dialesize=Parameter.EnergyBuffDetect_dialesize;
        dilate(bin,bin,getStructuringElement(MORPH_RECT,Size(dialesize,dialesize)));//膨胀
    }
    if(Parameter.EnergyBuffDetect_erodesize!=0){
        int &erodesize=Parameter.EnergyBuffDetect_erodesize;

        erode(bin,bin,getStructuringElement(MORPH_RECT,Size(erodesize,erodesize)));//腐蚀
    }

    vector<Vec4i> hierarchy;
    vector<vector<cv::Point> > contours;
    findContours(bin, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE);
    vector<EnergyBuffArmor> result_buff_armors;
    for(int i=0;i<contours.size();i++){
        double area=contourArea(contours[i], false);
        if(area>500&&area<3000){//面积筛选
            G_buffinformation.tmpparam0= -1;
            RotatedRect contour_rect=minAreaRect(contours[i]);
            if(contour_rect.size.width>90||contour_rect.size.height>90)continue;
            if(contour_rect.size.width<15||contour_rect.size.height<15)continue;
            G_buffinformation.tmpparam0= -2;

            if(hierarchy[i][3]==-1) {
                continue;
            } else{//有父级轮廓
//                all_rect=minAreaRect(contours[hierarchy[i][3]]);
//                if(all_rect.size.area()<area*5)continue;
            }
            G_buffinformation.tmpparam0= static_cast<float>(area);
            int hierarchy_judge=0;
            for(int qh=0;qh<2;qh++){//遍历同级轮廓
                for( int id=i;;id=hierarchy[id][qh]){
                    if(hierarchy[id][qh]!=-1){
                        if(contourArea(contours[hierarchy[id][qh]], false)>area/2)//如果同级轮廓有面积较大的
                            hierarchy_judge+=1;
                    } else{
                        break;
                    }
                }
            }

            EnergyBuffArmor tmp_buff_armor;
            tmp_buff_armor=EnergyBuffArmor(contour_rect, false);//以false的形式初始化EnergyBuffArmor
            Point2f circle_center=DectorCircle(tmp_buff_armor,contours,hierarchy,gray_img);//检测大神符圆心位置
            if(circle_center.x==-1){

                tmp_buff_armor=EnergyBuffArmor(contour_rect, true);//以false的形式初始化EnergyBuffArmor
                circle_center=DectorCircle(tmp_buff_armor,contours,hierarchy,color_img);//检测大神符圆心位置
            }
            if(circle_center.x==-1)continue;

            tmp_buff_armor.set_circle(circle_center);
//            EnergyBuffType energybuff_type=FlowWater;
//            vector<Point> hull;
//            convexHull(contours[hierarchy[i][3]],hull);
//            int area1= static_cast<int>(contourArea(hull));
//            if(fabs(area1)<14000){
//                energybuff_type=FlowWater;
//
//            } else{
//                energybuff_type=NoFlowWater;
//            }
            if(tmp_buff_armor.get_type()!=None){
//                tmp_buff_armor.set_type(energybuff_type);
                armors .push_back( tmp_buff_armor);
            }
        }
    }


#ifdef DEBUG
    if(G_setval.WORK){
        G_buffinformation.src=new Mat(src.clone());
        G_buffinformation.subcolor_img=new Mat(gray_img.clone());
        G_buffinformation.binary_img=new Mat(bin.clone());
    }
#endif
}

Point2f EnergyBuffDetector::DectorCircle(EnergyBuffArmor &buff_armor, const vector<vector<cv::Point>> &contours,vector<Vec4i> hierarchy ,Mat &gray_img)  {
    vector<Point2f> con(4);
    buff_armor.get_circle_rect_().points(con.data());
    vector<Mat> rgb_img;
    Point  circle_center(-1,-1);
    float grade=0;//最低分值
    for (int i=0;i<contours.size();i++){

        if(hierarchy[i][3]!=-1) {//有父级轮廓
            continue;
        }

        int area1= static_cast<int>(fabs(contourArea(contours[i])));
        if(area1>buff_armor.get_rotatedrect().size.area()/1.6  //面积
           ||area1<buff_armor.get_rotatedrect().size.area()/20)continue;
        G_buffinformation.tmpparam1=-1;

        Rect tem_rect=boundingRect(contours[i]);

        Point2f tmp_point=(tem_rect.tl()+tem_rect.br())/2;
        if(pointPolygonTest(con,tmp_point, false)!=1) continue;//判断点是否在区域内

        G_buffinformation.tmpparam1=area1;

        Mat r_img=gray_img(tem_rect);
        resize(r_img, r_img, Size(28,28));
        vector<float> descriptors;
        r_hog->compute(r_img, descriptors);
        Mat des=Mat(descriptors);
        transpose(des,des);
        Mat result,nearest,dist;
        knn_Model_r->findNearest(des,1,result,nearest,dist);
        float kd_r=dist.at<float>(0,0);
        buff_armor.set_kd_r(kd_r);
        buff_armor.set_circle(tmp_point);
        EnergyBuffType  bufftype=FlowWaterLight(buff_armor,gray_img);

        static int ucount_r=0;
        if(bufftype==None){
            continue;
        } else if (bufftype==FlowWater){
            Mat circle_img=gray_img(tem_rect);
            ucount_r++;
            buff_armor.set_type(FlowWater);
        }else{
            if(buff_armor.get_type()==FlowWater)continue;
            else buff_armor.set_type(NoFlowWater);
        }


        float tmpgrade=100;//总分
        float diffdistance=getDistance(tmp_point,buff_armor.br())-getDistance(tmp_point,buff_armor.bl());//圆心到左右两个顶点长度之差
        float rate1=getDistance(tmp_point,buff_armor.center())/buff_armor.get_width();//圆心到装甲板中心距离与装甲板宽之比
//        float rate2=buff_armor.get_rotatedrect().size.area()/tem_rect.size().area();//圆心面积与装甲板面积比
        tmpgrade-=fabs(diffdistance);
        tmpgrade-=10*kd_r;
        tmpgrade+=8*rate1;//得分
        if(tmpgrade>grade){
            grade=tmpgrade;
            buff_armor.set_kd_r(kd_r);
            circle_center=tmp_point;
        }

    }
    return circle_center;
}

EnergyBuffType  EnergyBuffDetector::FlowWaterLight(EnergyBuffArmor &buff_armor ,const Mat& gray_img){


    Mat armorimg;
    RotatedRect roi=buff_armor.get_ml_rect_();
    float h=max(roi.size.width,roi.size.height)*1.4f;
    Rect roi_mat=Rect(roi.center-Point2f(h/2,h/2),roi.center+Point2f(h/2,h/2));
    Rect roi_matcopy=Rect(roi_mat);
    if(RectSafety(roi_matcopy,gray_img.size())){
        Mat roiimg;
        if(roi_matcopy!=roi_mat){
            roiimg=Mat::zeros(roi_mat.size(),CV_8U);
            gray_img(roi_matcopy).copyTo(roiimg(Rect(roi_matcopy.tl()-roi_mat.tl(),roi_matcopy.size())));
        } else{
            roiimg=gray_img(roi_mat);
        }
        Mat rotationmat = getRotationMatrix2D(Point(roi.center)-roi_mat.tl(),(buff_armor.get_angle()+kPI/2)*180/kPI, 1);//仿射变换
        warpAffine(roiimg,armorimg,rotationmat,roiimg.size());
        Size size_;
        if(roi.size.width>roi.size.height){
            size_=roi.size;
        } else{
            size_=Size(cvRound(roi.size.height),cvRound(roi.size.width));
        }
        Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
        if(RectSafety(realsizerect,armorimg.size()))
        {  armorimg = armorimg(realsizerect);
        } else{
            armorimg = Mat::zeros(96,28,CV_8U);
        }
    } else{
        armorimg = Mat::zeros(96,28,CV_8U);
    }
    Mat bin;
    resize(armorimg,armorimg,Size(96,28));
    vector<float> descriptors;
    flowwater_hog->compute(armorimg, descriptors);
    Mat des=Mat(descriptors);
    transpose(des,des);
    Mat result,nearest,dist;
    knn_Model_flowwater->findNearest(des,1,result,nearest,dist);
    float kd_flowwater=dist.at<float>(0,0)*6.5f;
    buff_armor.set_kd_flowwater(kd_flowwater);


    threshold(armorimg, bin,0 ,255,THRESH_OTSU);
    erode(bin,bin,getStructuringElement(MORPH_RECT,Size(4,2)));//腐蚀
    vector<vector<cv::Point> > contours;

    findContours(bin, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);
    int contour_size=0;
    int big_contour_size=0;
//    float rh=max(roi.size.width,roi.size.height);
    float  rh=96;
    for(int i=0;i<contours.size();i++){
        Rect trect=boundingRect(contours[i]);
        if(trect.size().area()>roi.size.area()/3
           &&trect.size().width>rh*0.7){
            big_contour_size+=1;
        } else{

            if(trect.size().height>rh/12
               &&trect.size().height<rh/4
               &&1.2*trect.size().height>trect.size().width){
                contour_size++;
            } else{
                ;
            }

        }
    }
    static int ucount=0;
    if(kd_flowwater<5){
        return EnergyBuffType (FlowWater);
    }

    if(big_contour_size>0){
        G_buffinformation.tmpparam3=big_contour_size;
        return EnergyBuffType (NoFlowWater);
    } else{
        if(contour_size>4){
            G_buffinformation.tmpparam2=contour_size;
//          imwrite("/home/nathan/文档/国赛大符样本/flowwater/1/"+to_string(ucount)+".png",armorimg);
            ucount++;
            return EnergyBuffType (FlowWater);
        } else{
            return EnergyBuffType (None);
        }

    }


}

//void  EnergyBuffDetector::SetMlRoi(EnergyBuffArmor &buff_armor, const Mat &src) {//参照armordetect相应函数
//
//
//    Mat armorimg;
//
//    RotatedRect roi=buff_armor.get_ml_rect_();
//    float h=max(roi.size.width,roi.size.height)*1.4f;
//    Rect roi_mat=Rect(roi.center-Point2f(h/2,h/2),roi.center+Point2f(h/2,h/2));
//    Rect roi_matcopy=Rect(roi_mat);
//    if(RectSafety(roi_matcopy,src.size())){
//        Mat roiimg;
//        if(roi_matcopy!=roi_mat){
//            roiimg=Mat::zeros(roi_mat.size(),CV_8U);
//            src(roi_matcopy).copyTo(roiimg(Rect(roi_matcopy.tl()-roi_mat.tl(),roi_matcopy.size())));
//        } else{
//            roiimg=src(roi_mat);
//        }
//        Mat rotationmat = getRotationMatrix2D(Point(roi.center)-roi_mat.tl(),(buff_armor.get_angle()+kPI/2)*180/kPI, 1);//仿射变换
//        warpAffine(roiimg,armorimg,rotationmat,roiimg.size());
//        Size size_;
//        if(roi.size.width>roi.size.height){
//            size_=roi.size;
//        } else{
//            size_=Size(cvRound(roi.size.height),cvRound(roi.size.width));
//        }
//        Rect realsizerect(Point(roi.center)-roi_mat.tl()-Point(size_)/2,size_);
//        if(RectSafety(realsizerect,armorimg.size()))
//        {  armorimg = armorimg(realsizerect);
//        } else{
//            armorimg = Mat::zeros(96,28,CV_8U);
//        }
//    } else{
//        armorimg = Mat::zeros(96,28,CV_8U);
//    }
//    Mat rgb[3];
//    split(armorimg,rgb);
//    armorimg = rgb[my_color_];
//    resize(armorimg,armorimg,Size(96,28));
//    buff_armor.set_ml_image(armorimg);
//}